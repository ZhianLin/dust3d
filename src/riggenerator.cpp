#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <QGuiApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <cmath>
#include <QVector2D>
#include <queue>
#include <iostream>
#include <unordered_map>
#include "riggenerator.h"
#include "util.h"
#include "boundingboxmesh.h"
#include "theme.h"

class GroupEndpointsStitcher
{
public:
    GroupEndpointsStitcher(const std::vector<OutcomeNode> *nodes,
            const std::vector<std::unordered_set<size_t>> *groups,
            const std::vector<std::pair<size_t, size_t>> *groupEndpoints,
            std::vector<std::pair<size_t, float>> *stitchResult) :
        m_nodes(nodes),
        m_groups(groups),
        m_groupEndpoints(groupEndpoints),
        m_stitchResult(stitchResult)
    {
    }
    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            std::vector<std::pair<size_t, float>> distance2WithNodes;
            const auto &endpoint = (*m_groupEndpoints)[i];
            const auto &endpointNode = (*m_nodes)[endpoint.second];
            for (size_t groupIndex = 0; groupIndex < m_groups->size(); ++groupIndex) {
                if (endpoint.first == groupIndex)
                    continue;
                for (const auto &j: (*m_groups)[groupIndex]) {
                    const auto &node = (*m_nodes)[j];
                    if (node.partId == endpointNode.partId ||
                            node.mirroredByPartId == endpointNode.partId ||
                            node.mirrorFromPartId == endpointNode.partId) {
                        continue;
                    }
                    distance2WithNodes.push_back({j,
                        (endpointNode.origin - node.origin).lengthSquared()});
                }
            }
            if (distance2WithNodes.empty())
                continue;
            (*m_stitchResult)[i] = *std::min_element(distance2WithNodes.begin(), distance2WithNodes.end(), [](const std::pair<size_t, float> &first, const std::pair<size_t, float> &second) {
                return first.second < second.second;
            });
        }
    }
private:
    const std::vector<OutcomeNode> *m_nodes = nullptr;
    const std::vector<std::unordered_set<size_t>> *m_groups = nullptr;
    const std::vector<std::pair<size_t, size_t>> *m_groupEndpoints = nullptr;
    std::vector<std::pair<size_t, float>> *m_stitchResult = nullptr;
};

RigGenerator::RigGenerator(RigType rigType, const Outcome &outcome) :
    m_rigType(rigType),
    m_outcome(new Outcome(outcome))
{
}

RigGenerator::~RigGenerator()
{
    delete m_outcome;
    delete m_resultMesh;
    delete m_resultBones;
    delete m_resultWeights;
}

Outcome *RigGenerator::takeOutcome()
{
    Outcome *outcome = m_outcome;
    m_outcome = nullptr;
    return outcome;
}

std::vector<RiggerBone> *RigGenerator::takeResultBones()
{
    std::vector<RiggerBone> *resultBones = m_resultBones;
    m_resultBones = nullptr;
    return resultBones;
}

std::map<int, RiggerVertexWeights> *RigGenerator::takeResultWeights()
{
    std::map<int, RiggerVertexWeights> *resultWeights = m_resultWeights;
    m_resultWeights = nullptr;
    return resultWeights;
}

MeshLoader *RigGenerator::takeResultMesh()
{
    MeshLoader *resultMesh = m_resultMesh;
    m_resultMesh = nullptr;
    return resultMesh;
}

bool RigGenerator::isSucceed()
{
    return m_isSucceed;
}

const std::vector<std::pair<QtMsgType, QString>> &RigGenerator::messages()
{
    return m_messages;
}

void RigGenerator::groupNodeIndices(const std::map<size_t, std::unordered_set<size_t>> &neighborMap,
        std::vector<std::unordered_set<size_t>> *groups)
{
    std::unordered_set<size_t> visited;
    for (const auto &it: neighborMap) {
        if (visited.find(it.first) != visited.end())
            continue;
        std::unordered_set<size_t> group;
        std::queue<size_t> waitQueue;
        visited.insert(it.first);
        group.insert(it.first);
        waitQueue.push(it.first);
        while (!waitQueue.empty()) {
            auto nodeIndex = waitQueue.front();
            waitQueue.pop();
            auto findNeighbor = neighborMap.find(nodeIndex);
            if (findNeighbor != neighborMap.end()) {
                for (const auto &neighbor: findNeighbor->second) {
                    if (visited.find(neighbor) == visited.end()) {
                        visited.insert(neighbor);
                        group.insert(neighbor);
                        waitQueue.push(neighbor);
                    }
                }
            }
        }
        groups->push_back(group);
    }
}

void RigGenerator::buildNeighborMap()
{
    if (nullptr == m_outcome->triangleSourceNodes())
        return;
    
    std::map<std::pair<QUuid, QUuid>, size_t> nodeIdToIndexMap;
    for (size_t i = 0; i < m_outcome->bodyNodes.size(); ++i) {
        const auto &node = m_outcome->bodyNodes[i];
        nodeIdToIndexMap.insert({{node.partId, node.nodeId}, i});
        m_neighborMap.insert({i, {}});
    }
    
    for (const auto &it: m_outcome->bodyEdges) {
        const auto &findSource = nodeIdToIndexMap.find(it.first);
        if (findSource == nodeIdToIndexMap.end())
            continue;
        const auto &findTarget = nodeIdToIndexMap.find(it.second);
        if (findTarget == nodeIdToIndexMap.end())
            continue;
        m_neighborMap[findSource->second].insert(findTarget->second);
        m_neighborMap[findTarget->second].insert(findSource->second);
    }
    
    //std::vector<std::tuple<QVector3D, QVector3D, float, float, QColor>> debugBoxes;
    
    while (true) {
        std::vector<std::unordered_set<size_t>> groups;
        groupNodeIndices(m_neighborMap, &groups);
        if (groups.size() < 2)
            break;

        std::vector<std::pair<size_t, size_t>> groupEndpoints;
        for (size_t groupIndex = 0; groupIndex < groups.size(); ++groupIndex) {
            const auto &group = groups[groupIndex];
            for (const auto &nodeIndex: group) {
                if (m_neighborMap[nodeIndex].size() >= 2)
                    continue;
                groupEndpoints.push_back({groupIndex, nodeIndex});
            }
        }
        
        if (groupEndpoints.empty())
            break;
        
        std::vector<std::pair<size_t, float>> stitchResult(groupEndpoints.size(),
            {m_outcome->bodyNodes.size(), std::numeric_limits<float>::max()});
        tbb::parallel_for(tbb::blocked_range<size_t>(0, groupEndpoints.size()),
            GroupEndpointsStitcher(&m_outcome->bodyNodes, &groups, &groupEndpoints,
                &stitchResult));
        auto minDistantMatch = std::min_element(stitchResult.begin(), stitchResult.end(), [&](
                const std::pair<size_t, float> &first,
                const std::pair<size_t, float> &second) {
            return first.second < second.second;
        });
        if (minDistantMatch->first == m_outcome->bodyNodes.size())
            break;
        
        const auto &fromNodeIndex = groupEndpoints[minDistantMatch - stitchResult.begin()].second;
        const auto &toNodeIndex = minDistantMatch->first;
        m_neighborMap[fromNodeIndex].insert(toNodeIndex);
        m_neighborMap[toNodeIndex].insert(fromNodeIndex);
        
        //const auto &fromNode = m_outcome->bodyNodes[fromNodeIndex];
        //const auto &toNode = m_outcome->bodyNodes[toNodeIndex];
        //debugBoxes.push_back(std::make_tuple(fromNode.origin, toNode.origin,
        //    fromNode.radius, toNode.radius, Qt::red));
    }
    
    //m_debugEdgeVertices = buildBoundingBoxMeshEdges(debugBoxes, &m_debugEdgeVerticesNum);
}

void RigGenerator::removeBranchsFromNodes(const std::vector<std::vector<size_t>> *boneNodeIndices,
        std::vector<size_t> *resultNodes)
{
    resultNodes->resize(boneNodeIndices->size());
    for (size_t i = 0; i < boneNodeIndices->size(); ++i) {
        const auto &source = (*boneNodeIndices)[i];
        if (1 == source.size()) {
            (*resultNodes)[i] = source[0];
            continue;
        }
        bool foundJoint = false;
        std::vector<std::pair<size_t, float>> radisArray(source.size());
        for (size_t j = 0; j < source.size(); ++j) {
            const auto &bodyNode = m_outcome->bodyNodes[source[j]];
            if (BoneMark::None != bodyNode.boneMark) {
                foundJoint = true;
                (*resultNodes)[i] = source[j];
                break;
            }
            radisArray[j] = {
                source[j],
                bodyNode.radius
            };
        }
        if (foundJoint)
            continue;
        (*resultNodes)[i] = std::max_element(radisArray.begin(), radisArray.end(), [](const std::pair<size_t, float> &first,
                    const std::pair<size_t, float> &second) {
                return first.second < second.second;
            })->first;
    }
}

void RigGenerator::buildBoneNodeChain()
{
    std::vector<std::tuple<size_t, std::unordered_set<size_t>, bool>> segments;
    std::unordered_set<size_t> middle;
    size_t middleStartNodeIndex = m_outcome->bodyNodes.size();
    for (size_t nodeIndex = 0; nodeIndex < m_outcome->bodyNodes.size(); ++nodeIndex) {
        const auto &node = m_outcome->bodyNodes[nodeIndex];
        if (!BoneMarkIsBranchNode(node.boneMark))
            continue;
        m_branchNodesMapByMark[(int)node.boneMark].push_back(nodeIndex);
        if (BoneMark::Neck == node.boneMark) {
            if (middleStartNodeIndex == m_outcome->bodyNodes.size())
                middleStartNodeIndex = nodeIndex;
        } else if (BoneMark::Tail == node.boneMark) {
            middleStartNodeIndex = nodeIndex;
        }
        std::unordered_set<size_t> left;
        std::unordered_set<size_t> right;
        splitByNodeIndex(nodeIndex, &left, &right);
        if (left.size() > right.size())
            std::swap(left, right);
        for (const auto &it: right)
            middle.insert(it);
        segments.push_back({nodeIndex, left, false});
    }
    for (const auto &it: segments) {
        const auto &nodeIndex = std::get<0>(it);
        const auto &left = std::get<1>(it);
        for (const auto &it: left)
            middle.erase(it);
        middle.erase(nodeIndex);
    }
    middle.erase(middleStartNodeIndex);
    if (middleStartNodeIndex != m_outcome->bodyNodes.size())
        segments.push_back({middleStartNodeIndex, middle, true});
    for (const auto &it: segments) {
        const auto &fromNodeIndex = std::get<0>(it);
        const auto &left = std::get<1>(it);
        const auto &isSpine = std::get<2>(it);
        const auto &fromNode = m_outcome->bodyNodes[fromNodeIndex];
        std::vector<std::vector<size_t>> boneNodeIndices;
        std::unordered_set<size_t> visited;
        std::vector<size_t> boneNodeChain;
        std::vector<bool> isJointFlags;
        size_t attachNodeIndex = fromNodeIndex;
        collectNodesForBoneRecursively(fromNodeIndex,
            &left,
            &boneNodeIndices,
            0,
            &visited);
        if (BoneMark::Limb == fromNode.boneMark) {
            for (const auto &neighbor: m_neighborMap[fromNodeIndex]) {
                if (left.find(neighbor) == left.end()) {
                    attachNodeIndex = neighbor;
                    break;
                }
            }
        }
        removeBranchsFromNodes(&boneNodeIndices, &boneNodeChain);
        //if (boneNodeChain.size() >= 3) {
        //    const auto &headNode = m_outcome->bodyNodes[boneNodeChain[0]];
        //    const auto &firstNode = m_outcome->bodyNodes[boneNodeChain[boneNodeChain.size() - 2]];
        //    const auto &secondNode = m_outcome->bodyNodes[boneNodeChain[boneNodeChain.size() - 1]];
        //    if ((secondNode.origin - headNode.origin).lengthSquared() <=
        //            (firstNode.origin - headNode.origin).lengthSquared()) {
        //        boneNodeChain.resize(boneNodeChain.size() - 1);
        //    }
        //}
        isJointFlags.resize(boneNodeIndices.size(), false);
        for (size_t i = 0; i < boneNodeIndices.size(); ++i) {
            for (const auto &nodeIndex: boneNodeIndices[i]) {
                if (BoneMark::None == m_outcome->bodyNodes[nodeIndex].boneMark)
                    continue;
                isJointFlags[i] = true;
                break;
            }
        }
        m_boneNodeChain.push_back({fromNodeIndex, boneNodeIndices, boneNodeChain, isSpine, isJointFlags, attachNodeIndex});
    }
    for (size_t i = 0; i < m_boneNodeChain.size(); ++i) {
        const auto &chain = m_boneNodeChain[i];
        const auto &node = m_outcome->bodyNodes[chain.fromNodeIndex];
        const auto &isSpine = chain.isSpine;
        //printf("Chain[%lu] %s %s", i, BoneMarkToString(node.boneMark), isSpine ? "SPINE " : "");
        //printf("|");
        //for (size_t j = 0; j < chain.nodeChain.size(); ++j) {
        //    printf("%lu%s ", chain.nodeChain[j], chain.nodeIsJointFlags[j] ? "(JOINT)" : "");
        //}
        //printf("\r\n");
        //for (size_t j = 0; j < chain.nodeChain.size(); ++j) {
        //    const auto &node = m_outcome->bodyNodes[chain.nodeChain[j]];
        //    printf("         >>%lu part:%s node:%s (%f,%f,%f)%s\r\n",
        //        chain.nodeChain[j],
        //        node.partId.toString().toUtf8().constData(),
        //        node.nodeId.toString().toUtf8().constData(),
        //        node.origin.x(), node.origin.y(), node.origin.z(),
        //        chain.nodeIsJointFlags[j] ? "(JOINT)" : "");
        //}
        if (isSpine) {
            m_spineChains.push_back(i);
            continue;
        }
        if (BoneMark::Neck == node.boneMark) {
            m_neckChains.push_back(i);
        } else if (BoneMark::Tail == node.boneMark) {
            m_tailChains.push_back(i);
        } else if (BoneMark::Limb == node.boneMark) {
            if (node.origin.x() < 0) {
                m_leftLimbChains.push_back(i);
            } else if (node.origin.x() > 0) {
                m_rightLimbChains.push_back(i);
            }
        }
    }
}

void RigGenerator::calculateSpineDirection(bool *isVertical)
{
    float left = std::numeric_limits<float>::lowest();
    float right = std::numeric_limits<float>::max();
    float top = std::numeric_limits<float>::lowest();
    float bottom = std::numeric_limits<float>::max();
    auto updateBoundingBox = [&](const std::vector<size_t> &chains) {
        for (const auto &it: chains) {
            const auto &node = m_outcome->bodyNodes[m_boneNodeChain[it].fromNodeIndex];
            if (node.origin.y() > top)
                top = node.origin.y();
            if (node.origin.y() < bottom)
                bottom = node.origin.y();
            if (node.origin.z() > left)
                left = node.origin.z();
            if (node.origin.z() < right)
                right = node.origin.z();
        }
    };
    updateBoundingBox(m_leftLimbChains);
    updateBoundingBox(m_rightLimbChains);
    auto zLength = left - right;
    auto yLength = top - bottom;
    *isVertical = yLength >= zLength;
}

void RigGenerator::attachLimbsToSpine()
{
    Q_ASSERT(m_leftLimbChains.size() == m_rightLimbChains.size());
    Q_ASSERT(m_spineChains.size() == 1);
    
    m_attachLimbsToSpineChainPositions.resize(m_leftLimbChains.size());
    for (size_t i = 0; i < m_leftLimbChains.size(); ++i) {
        const auto &leftNode = m_outcome->bodyNodes[m_boneNodeChain[m_leftLimbChains[i]].attachNodeIndex];
        const auto &rightNode = m_outcome->bodyNodes[m_boneNodeChain[m_rightLimbChains[i]].attachNodeIndex];
        auto limbMiddle = (leftNode.origin + rightNode.origin) * 0.5;
        std::vector<std::pair<size_t, float>> distance2WithSpine;
        auto boneNodeChainIndex = m_spineChains[0];
        const auto &nodeIndices = m_boneNodeChain[boneNodeChainIndex].nodeChain;
        distance2WithSpine.reserve(nodeIndices.size());
        for (size_t j = 0; j < nodeIndices.size(); ++j) {
            const auto &nodeIndex = nodeIndices[j];
            distance2WithSpine.push_back({
                j,
                (m_outcome->bodyNodes[nodeIndex].origin - limbMiddle).lengthSquared()
            });
        }
        if (distance2WithSpine.empty())
            continue;
        auto chainPos = std::min_element(distance2WithSpine.begin(), distance2WithSpine.end(), [](
                const std::pair<size_t, float> &first, const std::pair<size_t, float> &second) {
            return first.second < second.second;
        })->first;
        m_attachLimbsToSpineChainPositions[i] = chainPos;
    }
}

void RigGenerator::buildSkeleton()
{
    bool addMarkHelpInfo = false;
    
    if (m_leftLimbChains.size() != m_rightLimbChains.size()) {
        m_messages.push_back({QtInfoMsg, tr("Imbalanced left and right limbs")});
    } else if (m_leftLimbChains.empty()) {
        m_messages.push_back({QtInfoMsg, tr("No limbs found")});
        addMarkHelpInfo = true;
    }
    
    if (m_spineChains.empty()) {
        m_messages.push_back({QtInfoMsg, tr("No body found")});
        addMarkHelpInfo = true;
    }
    
    if (addMarkHelpInfo) {
        m_messages.push_back({QtInfoMsg, tr("Please mark the neck, limbs and joints from the context menu")});
    }
    
    if (!m_messages.empty())
        return;
    
    calculateSpineDirection(&m_isSpineVertical);
    
    auto sortLimbChains = [&](std::vector<size_t> &chains) {
        std::sort(chains.begin(), chains.end(), [&](const size_t &first,
                const size_t &second) {
            if (m_isSpineVertical) {
                return m_outcome->bodyNodes[m_boneNodeChain[first].fromNodeIndex].origin.y() <
                    m_outcome->bodyNodes[m_boneNodeChain[second].fromNodeIndex].origin.y();
            }
            return m_outcome->bodyNodes[m_boneNodeChain[first].fromNodeIndex].origin.z() <
                m_outcome->bodyNodes[m_boneNodeChain[second].fromNodeIndex].origin.z();
        });
    };
    sortLimbChains(m_leftLimbChains);
    sortLimbChains(m_rightLimbChains);
    
    attachLimbsToSpine();
    extractSpineJoints();
    extractBranchJoints();
    
    size_t rootSpineJointIndex = m_attachLimbsToSpineJointIndices[0];
    size_t lastSpineJointIndex = m_spineJoints.size() - 1;
    
    m_resultBones = new std::vector<RiggerBone>;
    m_resultWeights = new std::map<int, RiggerVertexWeights>;
    
    {
        const auto &firstSpineNode = m_outcome->bodyNodes[m_spineJoints[rootSpineJointIndex]];
        RiggerBone bone;
        bone.headPosition = QVector3D(0.0, 0.0, 0.0);
        bone.tailPosition = firstSpineNode.origin;
        bone.headRadius = 0;
        bone.tailRadius = firstSpineNode.radius;
        bone.name = QString("Body");
        bone.index = m_resultBones->size();
        m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
        m_resultBones->push_back(bone);
    }
    
    auto attachedBoneIndex = [&](size_t spineJointIndex) {
        if (spineJointIndex == rootSpineJointIndex) {
            return m_boneNameToIndexMap[QString("Body")];
        }
        return m_boneNameToIndexMap[QString("Spine") + QString::number(spineJointIndex - rootSpineJointIndex)];
    };
    
    for (size_t spineJointIndex = rootSpineJointIndex;
            spineJointIndex + 1 < m_spineJoints.size();
            ++spineJointIndex) {
        const auto &currentNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex]];
        const auto &nextNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex + 1]];
        RiggerBone bone;
        bone.headPosition = currentNode.origin;
        bone.tailPosition = nextNode.origin;
        bone.headRadius = currentNode.radius;
        bone.tailRadius = nextNode.radius;
        bone.color = Theme::white;
        bone.name = QString("Spine") + QString::number(spineJointIndex + 1 - rootSpineJointIndex);
        bone.index = m_resultBones->size();
        m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
        m_resultBones->push_back(bone);
        (*m_resultBones)[attachedBoneIndex(spineJointIndex)].children.push_back(bone.index);
    }
    
    auto addSpineLinkBone = [&](size_t limbIndex,
            const std::vector<std::vector<size_t>> &limbJoints,
            const QString &chainPrefix) {
        QString chainName = chainPrefix + QString::number(limbIndex + 1);
        const auto &spineJointIndex = m_attachLimbsToSpineJointIndices[limbIndex];
        const auto &spineNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex]];
        const auto &limbFirstNode = m_outcome->bodyNodes[limbJoints[limbIndex][0]];
        const auto &parentIndex = attachedBoneIndex(spineJointIndex);;
        RiggerBone bone;
        bone.headPosition = spineNode.origin;
        bone.tailPosition = limbFirstNode.origin;
        bone.headRadius = spineNode.radius;
        bone.tailRadius = limbFirstNode.radius;
        bone.color = Theme::white;
        bone.name = QString("Virtual_") + (*m_resultBones)[parentIndex].name + QString("_") + chainName;
        bone.index = m_resultBones->size();
        m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
        m_resultBones->push_back(bone);
        (*m_resultBones)[parentIndex].children.push_back(bone.index);
    };
    
    auto addLimbBone = [&](size_t limbIndex,
            const std::vector<std::vector<size_t>> &limbJoints,
            const QString &chainPrefix) {
        const auto &joints = limbJoints[limbIndex];
        QString chainName = chainPrefix + QString::number(limbIndex + 1);
        for (size_t limbJointIndex = 0;
                limbJointIndex + 1 < joints.size();
                ++limbJointIndex) {
            const auto &currentNode = m_outcome->bodyNodes[joints[limbJointIndex]];
            const auto &nextNode = m_outcome->bodyNodes[joints[limbJointIndex + 1]];
            RiggerBone bone;
            bone.headPosition = currentNode.origin;
            bone.tailPosition = nextNode.origin;
            bone.headRadius = currentNode.radius;
            bone.tailRadius = nextNode.radius;
            bone.color = 0 == limbJointIndex % 2 ? BoneMarkToColor(BoneMark::Limb) : BoneMarkToColor(BoneMark::Joint);
            bone.name = chainName + QString("_Joint") + QString::number(limbJointIndex + 1);
            bone.index = m_resultBones->size();
            m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (limbJointIndex > 0) {
                auto parentName = chainName + QString("_Joint") + QString::number(limbJointIndex);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            } else {
                const auto &spineJointIndex = m_attachLimbsToSpineJointIndices[limbIndex];
                const auto &parentIndex = attachedBoneIndex(spineJointIndex);
                auto parentName = QString("Virtual_") + (*m_resultBones)[parentIndex].name + QString("_") + chainName;
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    };
    
    for (size_t limbIndex = 0;
            limbIndex < m_attachLimbsToSpineJointIndices.size();
            ++limbIndex) {
        addSpineLinkBone(limbIndex, m_leftLimbJoints, QString("LeftLimb"));
        addSpineLinkBone(limbIndex, m_rightLimbJoints, QString("RightLimb"));
        addLimbBone(limbIndex, m_leftLimbJoints, QString("LeftLimb"));
        addLimbBone(limbIndex, m_rightLimbJoints, QString("RightLimb"));
    }
    
    if (!m_neckJoints.empty()) {
        for (size_t neckJointIndex = 0;
                neckJointIndex + 1 < m_neckJoints.size();
                ++neckJointIndex) {
            const auto &currentNode = m_outcome->bodyNodes[m_neckJoints[neckJointIndex]];
            const auto &nextNode = m_outcome->bodyNodes[m_neckJoints[neckJointIndex + 1]];
            RiggerBone bone;
            bone.headPosition = currentNode.origin;
            bone.tailPosition = nextNode.origin;
            bone.headRadius = currentNode.radius;
            bone.tailRadius = nextNode.radius;
            bone.color = BoneMarkToColor(BoneMark::Neck);
            bone.name = QString("Neck_Joint") + QString::number(neckJointIndex + 1);
            bone.index = m_resultBones->size();
            m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (neckJointIndex > 0) {
                auto parentName = QString("Neck_Joint") + QString::number(neckJointIndex);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            } else {
                auto parentName = QString("Spine") + QString::number(lastSpineJointIndex - rootSpineJointIndex);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    }
    
    if (!m_tailJoints.empty()) {
        for (int spineJointIndex = rootSpineJointIndex;
                spineJointIndex >= 0;
                --spineJointIndex) {
            const auto &currentNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex]];
            const auto &nextNode = spineJointIndex > 0 ?
                m_outcome->bodyNodes[m_spineJoints[spineJointIndex - 1]] :
                m_outcome->bodyNodes[m_tailJoints[0]];
            RiggerBone bone;
            bone.headPosition = currentNode.origin;
            bone.tailPosition = nextNode.origin;
            bone.headRadius = currentNode.radius;
            bone.tailRadius = nextNode.radius;
            bone.color = Theme::white;
            bone.name = QString("Spine0") + QString::number(rootSpineJointIndex - spineJointIndex + 1);
            bone.index = m_resultBones->size();
            m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if ((int)rootSpineJointIndex == spineJointIndex) {
                auto parentName = QString("Body");
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            } else {
                auto parentName = QString("Spine0") + QString::number(rootSpineJointIndex - spineJointIndex);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    
        for (size_t tailJointIndex = 0;
                tailJointIndex + 1 < m_tailJoints.size();
                ++tailJointIndex) {
            const auto &currentNode = m_outcome->bodyNodes[m_tailJoints[tailJointIndex]];
            const auto &nextNode = m_outcome->bodyNodes[m_tailJoints[tailJointIndex + 1]];
            RiggerBone bone;
            bone.headPosition = currentNode.origin;
            bone.tailPosition = nextNode.origin;
            bone.headRadius = currentNode.radius;
            bone.tailRadius = nextNode.radius;
            bone.color = 0 == tailJointIndex % 2 ? BoneMarkToColor(BoneMark::Tail) : BoneMarkToColor(BoneMark::Joint);
            bone.name = QString("Tail_Joint") + QString::number(tailJointIndex + 1);
            bone.index = m_resultBones->size();
            m_boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (tailJointIndex > 0) {
                auto parentName = QString("Tail_Joint") + QString::number(tailJointIndex);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            } else {
                auto parentName = QString("Spine0") + QString::number(rootSpineJointIndex + 1);
                (*m_resultBones)[m_boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    }
    
    m_isSucceed = true;
}

void RigGenerator::computeSkinWeights()
{
    if (!m_isSucceed)
        return;

    auto collectNodeIndices = [&](size_t chainIndex,
            std::unordered_map<size_t, size_t> *nodeIndexToContainerMap,
            size_t containerIndex) {
        const auto &chain = m_boneNodeChain[chainIndex];
        for (const auto &it: chain.nodeIndices) {
            for (const auto &subIt: it)
                nodeIndexToContainerMap->insert({subIt, containerIndex});
        }
        nodeIndexToContainerMap->insert({chain.fromNodeIndex, containerIndex});
    };
    
    const size_t neckIndex = 0;
    const size_t tailIndex = 1;
    const size_t spineIndex = 2;
    const size_t limbStartIndex = 3;
    
    std::unordered_map<size_t, size_t> nodeIndicesToBranchMap;
    
    if (!m_neckChains.empty())
        collectNodeIndices(m_neckChains[0], &nodeIndicesToBranchMap, neckIndex);
    
    if (!m_tailChains.empty())
        collectNodeIndices(m_tailChains[0], &nodeIndicesToBranchMap, tailIndex);

    if (!m_spineChains.empty())
        collectNodeIndices(m_spineChains[0], &nodeIndicesToBranchMap, spineIndex);
    
    for (size_t i = 0; i < m_leftLimbChains.size(); ++i) {
        collectNodeIndices(m_leftLimbChains[i], &nodeIndicesToBranchMap,
            limbStartIndex + i);
    }
    
    for (size_t i = 0; i < m_rightLimbChains.size(); ++i) {
        collectNodeIndices(m_rightLimbChains[i], &nodeIndicesToBranchMap,
            limbStartIndex + m_leftLimbChains.size() + i);
    }
    
    std::vector<std::vector<size_t>> vertexBranches(limbStartIndex +
        m_leftLimbChains.size() +
        m_rightLimbChains.size() +
        1);
    
    std::map<std::pair<QUuid, QUuid>, size_t> nodeIdToIndexMap;
    for (size_t nodeIndex = 0; nodeIndex < m_outcome->bodyNodes.size(); ++nodeIndex) {
        const auto &node = m_outcome->bodyNodes[nodeIndex];
        nodeIdToIndexMap[{node.partId, node.nodeId}] = nodeIndex;
    }
    for (size_t vertexIndex = 0; vertexIndex < m_outcome->vertices.size(); ++vertexIndex) {
        const auto &vertexSourceId = m_outcome->vertexSourceNodes[vertexIndex];
        auto findNodeIndex = nodeIdToIndexMap.find(vertexSourceId);
        if (findNodeIndex == nodeIdToIndexMap.end()) {
            vertexBranches[spineIndex].push_back(vertexIndex);
            continue;
        }
        const auto &nodeIndex = findNodeIndex->second;
        auto findBranch = nodeIndicesToBranchMap.find(nodeIndex);
        if (findBranch == nodeIndicesToBranchMap.end()) {
            vertexBranches[spineIndex].push_back(vertexIndex);
            continue;
        }
        vertexBranches[findBranch->second].push_back(vertexIndex);
    }
    
    auto findNeckBoneIndex = m_boneNameToIndexMap.find(QString("Neck_Joint1"));
    if (findNeckBoneIndex != m_boneNameToIndexMap.end()) {
        computeBranchSkinWeights(findNeckBoneIndex->second,
            QString("Neck_"), vertexBranches[neckIndex],
            &vertexBranches[spineIndex]);
    }
    
    auto findTailBoneIndex = m_boneNameToIndexMap.find(QString("Tail_Joint1"));
    if (findTailBoneIndex != m_boneNameToIndexMap.end()) {
        computeBranchSkinWeights(findTailBoneIndex->second,
            QString("Tail_"), vertexBranches[tailIndex],
            &vertexBranches[spineIndex]);
    }
    
    for (size_t i = 0; i < m_leftLimbChains.size(); ++i) {
        auto namePrefix = QString("LeftLimb") + QString::number(i + 1) + "_";
        auto findLimbBoneIndex = m_boneNameToIndexMap.find(namePrefix + "Joint1");
        if (findLimbBoneIndex != m_boneNameToIndexMap.end()) {
            computeBranchSkinWeights(findLimbBoneIndex->second,
                namePrefix, vertexBranches[limbStartIndex + i],
                &vertexBranches[spineIndex]);
        }
    }
    
    for (size_t i = 0; i < m_rightLimbChains.size(); ++i) {
        auto namePrefix = QString("RightLimb") + QString::number(i + 1) + "_";
        auto findLimbBoneIndex = m_boneNameToIndexMap.find(namePrefix + "Joint1");
        if (findLimbBoneIndex != m_boneNameToIndexMap.end()) {
            computeBranchSkinWeights(findLimbBoneIndex->second,
                namePrefix, vertexBranches[limbStartIndex + m_leftLimbChains.size() + i],
                &vertexBranches[spineIndex]);
        }
    }
    
    auto findSpineBoneIndex = m_boneNameToIndexMap.find(QString("Spine1"));
    if (findSpineBoneIndex != m_boneNameToIndexMap.end()) {
        computeBranchSkinWeights(findSpineBoneIndex->second,
            QString("Spine"), vertexBranches[spineIndex]);
    }
    
    auto findBackSpineBoneIndex = m_boneNameToIndexMap.find(QString("Spine01"));
    if (findBackSpineBoneIndex != m_boneNameToIndexMap.end()) {
        computeBranchSkinWeights(findBackSpineBoneIndex->second,
            QString("Spine"), vertexBranches[spineIndex]);
    }
    
    for (auto &it: *m_resultWeights)
        it.second.finalizeWeights();
    
    //for (size_t i = 0; i < m_outcome->vertices.size(); ++i) {
    //    auto findWeights = m_resultWeights->find(i);
    //    if (findWeights == m_resultWeights->end()) {
    //        const auto &sourceNode = m_outcome->vertexSourceNodes[i];
    //        printf("NoWeight vertex index:%lu Source:%s %s\r\n",
    //            i,
    //            sourceNode.first.toString().toUtf8().constData(),
    //            sourceNode.second.toString().toUtf8().constData());
    //    }
    //}
}

void RigGenerator::computeBranchSkinWeights(size_t fromBoneIndex,
        const QString &boneNamePrefix,
        const std::vector<size_t> &vertexIndices,
        std::vector<size_t> *discardedVertexIndices)
{
    size_t boneIndex = fromBoneIndex;
    const auto &bone = (*m_resultBones)[boneIndex];
    std::vector<size_t> remainVertexIndices;
    auto forward = (bone.tailPosition - bone.headPosition).normalized();
    for (const auto &vertexIndex: vertexIndices) {
        const auto &position = m_outcome->vertices[vertexIndex];
        auto direction = (position - bone.headPosition).normalized();
        if (QVector3D::dotProduct(direction, forward) >= 0) {
            remainVertexIndices.push_back(vertexIndex);
        } else {
            if (nullptr != discardedVertexIndices)
                discardedVertexIndices->push_back(vertexIndex);
        }
    }
    while (true) {
        const auto &bone = (*m_resultBones)[boneIndex];
        if (!bone.name.startsWith(boneNamePrefix))
            break;
        if (bone.children.empty()) {
            for (const auto &vertexIndex: remainVertexIndices) {
                (*m_resultWeights)[vertexIndex].addBone(boneIndex, 1.0);
            }
            break;
        }
        const auto &child = (*m_resultBones)[bone.children[0]];
        auto forward = (((bone.tailPosition - bone.headPosition).normalized() +
            (child.tailPosition - child.headPosition).normalized()) * 0.5).normalized();
        std::vector<size_t> newRemainVertexIndices;
        for (const auto &vertexIndex: remainVertexIndices) {
            const auto &position = m_outcome->vertices[vertexIndex];
            auto direction = (position - bone.tailPosition).normalized();
            if (QVector3D::dotProduct(direction, forward) > 0) {
                newRemainVertexIndices.push_back(vertexIndex);
            } else {
                for (const auto &vertexIndex: remainVertexIndices) {
                    (*m_resultWeights)[vertexIndex].addBone(boneIndex, 1.0);
                }
            }
        }
        remainVertexIndices = newRemainVertexIndices;
        boneIndex = bone.children[0];
    }
}

void RigGenerator::extractBranchJoints()
{
    auto extractJoints = [&](const BoneNoeChain &chain, std::vector<size_t> *joints) {
        joints->push_back(chain.fromNodeIndex);
        for (size_t i = 0; i < chain.nodeChain.size(); ++i) {
            if (chain.nodeIsJointFlags[i] || i + 1 == chain.nodeChain.size())
                joints->push_back(chain.nodeChain[i]);
        }
    };
    if (!m_neckChains.empty())
        extractJoints(m_boneNodeChain[m_neckChains[0]], &m_neckJoints);
    if (!m_tailChains.empty())
        extractJoints(m_boneNodeChain[m_tailChains[0]], &m_tailJoints);
    m_leftLimbJoints.resize(m_leftLimbChains.size());
    for (size_t i = 0; i < m_leftLimbChains.size(); ++i) {
        extractJoints(m_boneNodeChain[m_leftLimbChains[i]], &m_leftLimbJoints[i]);
    }
    m_rightLimbJoints.resize(m_rightLimbChains.size());
    for (size_t i = 0; i < m_rightLimbChains.size(); ++i) {
        extractJoints(m_boneNodeChain[m_rightLimbChains[i]], &m_rightLimbJoints[i]);
    }
}

void RigGenerator::extractSpineJoints()
{
    auto &spine = m_boneNodeChain[m_spineChains[0]];
    auto findTail = m_branchNodesMapByMark.find((int)BoneMark::Tail);
    if (findTail != m_branchNodesMapByMark.end()) {
        m_spineJoints.push_back(findTail->second[0]);
    } else {
        std::reverse(spine.nodeChain.begin(), spine.nodeChain.end());
        std::reverse(spine.nodeIsJointFlags.begin(), spine.nodeIsJointFlags.end());
        for (auto &it: m_attachLimbsToSpineChainPositions) {
            it = spine.nodeChain.size() - 1 - it;
        }
    }
    m_attachLimbsToSpineJointIndices.resize(m_attachLimbsToSpineChainPositions.size());
    for (size_t i = 0; i < spine.nodeChain.size(); ++i) {
        bool limbsAttached = false;
        for (size_t j = 0; j < m_attachLimbsToSpineChainPositions.size(); ++j) {
            if (i == m_attachLimbsToSpineChainPositions[j]) {
                m_attachLimbsToSpineJointIndices[j] = m_spineJoints.size();
                limbsAttached = true;
            }
        }
        if (limbsAttached || spine.nodeIsJointFlags[i]) {
            m_spineJoints.push_back(spine.nodeChain[i]);
            continue;
        }
    }
    auto findNeck = m_branchNodesMapByMark.find((int)BoneMark::Neck);
    if (findNeck != m_branchNodesMapByMark.end()) {
        m_spineJoints.push_back(findNeck->second[0]);
    }
}

void RigGenerator::splitByNodeIndex(size_t nodeIndex,
        std::unordered_set<size_t> *left,
        std::unordered_set<size_t> *right)
{
    const auto &neighbors = m_neighborMap[nodeIndex];
    if (2 != neighbors.size()) {
        return;
    }
    {
        std::unordered_set<size_t> visited;
        visited.insert(*neighbors.begin());
        collectNodes(nodeIndex, left, &visited);
        left->erase(nodeIndex);
    }
    {
        std::unordered_set<size_t> visited;
        visited.insert(*(++neighbors.begin()));
        collectNodes(nodeIndex, right, &visited);
        right->erase(nodeIndex);
    }
}

void RigGenerator::collectNodes(size_t fromNodeIndex,
        std::unordered_set<size_t> *container,
        std::unordered_set<size_t> *visited)
{
    std::queue<size_t> waitQueue;
    waitQueue.push(fromNodeIndex);
    while (!waitQueue.empty()) {
        auto nodeIndex = waitQueue.front();
        waitQueue.pop();
        if (visited->find(nodeIndex) != visited->end())
            continue;
        visited->insert(nodeIndex);
        container->insert(nodeIndex);
        for (const auto &neighborNodeIndex: m_neighborMap[nodeIndex]) {
            if (visited->find(neighborNodeIndex) != visited->end())
                continue;
            waitQueue.push(neighborNodeIndex);
        }
    }
}

void RigGenerator::collectNodesForBoneRecursively(size_t fromNodeIndex,
        const std::unordered_set<size_t> *limitedNodeIndices,
        std::vector<std::vector<size_t>> *boneNodeIndices,
        size_t depth,
        std::unordered_set<size_t> *visited)
{
    std::vector<size_t> nodeIndices;
    for (const auto &nodeIndex: m_neighborMap[fromNodeIndex]) {
        if (limitedNodeIndices->find(nodeIndex) == limitedNodeIndices->end())
            continue;
        if (visited->find(nodeIndex) != visited->end())
            continue;
        visited->insert(nodeIndex);
        if (depth >= boneNodeIndices->size())
            boneNodeIndices->resize(depth + 1);
        (*boneNodeIndices)[depth].push_back(nodeIndex);
        nodeIndices.push_back(nodeIndex);
    }
    
    for (const auto &nodeIndex: nodeIndices) {
        collectNodesForBoneRecursively(nodeIndex,
            limitedNodeIndices,
            boneNodeIndices,
            depth + 1,
            visited);
    }
}

void RigGenerator::buildDemoMesh()
{
    // Blend vertices colors according to bone weights
    
    std::vector<QColor> inputVerticesColors(m_outcome->vertices.size(), Qt::black);
    if (m_isSucceed) {
        const auto &resultWeights = *m_resultWeights;
        const auto &resultBones = *m_resultBones;
        
        m_resultWeights = new std::map<int, RiggerVertexWeights>;
        *m_resultWeights = resultWeights;
        
        m_resultBones = new std::vector<RiggerBone>;
        *m_resultBones = resultBones;
        
        for (const auto &weightItem: resultWeights) {
            size_t vertexIndex = weightItem.first;
            const auto &weight = weightItem.second;
            int blendR = 0, blendG = 0, blendB = 0;
            for (int i = 0; i < 4; i++) {
                int boneIndex = weight.boneIndices[i];
                if (boneIndex > 0) {
                    const auto &bone = resultBones[boneIndex];
                    blendR += bone.color.red() * weight.boneWeights[i];
                    blendG += bone.color.green() * weight.boneWeights[i];
                    blendB += bone.color.blue() * weight.boneWeights[i];
                }
            }
            QColor blendColor = QColor(blendR, blendG, blendB, 255);
            inputVerticesColors[vertexIndex] = blendColor;
        }
    }
    
    // Create mesh for demo
    
    const std::vector<QVector3D> *triangleTangents = m_outcome->triangleTangents();
    const auto &inputVerticesPositions = m_outcome->vertices;
    const std::vector<std::vector<QVector3D>> *triangleVertexNormals = m_outcome->triangleVertexNormals();
    
    ShaderVertex *triangleVertices = nullptr;
    int triangleVerticesNum = 0;
    if (m_isSucceed) {
        triangleVertices = new ShaderVertex[m_outcome->triangles.size() * 3];
        const QVector3D defaultUv = QVector3D(0, 0, 0);
        const QVector3D defaultTangents = QVector3D(0, 0, 0);
        for (size_t triangleIndex = 0; triangleIndex < m_outcome->triangles.size(); triangleIndex++) {
            const auto &sourceTriangle = m_outcome->triangles[triangleIndex];
            const auto *sourceTangent = &defaultTangents;
            if (nullptr != triangleTangents)
                sourceTangent = &(*triangleTangents)[triangleIndex];
            for (int i = 0; i < 3; i++) {
                ShaderVertex &currentVertex = triangleVertices[triangleVerticesNum++];
                const auto &sourcePosition = inputVerticesPositions[sourceTriangle[i]];
                const auto &sourceColor = inputVerticesColors[sourceTriangle[i]];
                const auto *sourceNormal = &defaultUv;
                if (nullptr != triangleVertexNormals)
                    sourceNormal = &(*triangleVertexNormals)[triangleIndex][i];
                currentVertex.posX = sourcePosition.x();
                currentVertex.posY = sourcePosition.y();
                currentVertex.posZ = sourcePosition.z();
                currentVertex.texU = 0;
                currentVertex.texV = 0;
                currentVertex.colorR = sourceColor.redF();
                currentVertex.colorG = sourceColor.greenF();
                currentVertex.colorB = sourceColor.blueF();
                currentVertex.normX = sourceNormal->x();
                currentVertex.normY = sourceNormal->y();
                currentVertex.normZ = sourceNormal->z();
                currentVertex.metalness = MeshLoader::m_defaultMetalness;
                currentVertex.roughness = MeshLoader::m_defaultRoughness;
                currentVertex.tangentX = sourceTangent->x();
                currentVertex.tangentY = sourceTangent->y();
                currentVertex.tangentZ = sourceTangent->z();
            }
        }
    }
    
    // Create bone bounding box for demo
    
    ShaderVertex *edgeVertices = nullptr;
    int edgeVerticesNum = 0;
    
    if (m_isSucceed) {
        const auto &resultBones = *m_resultBones;
        std::vector<std::tuple<QVector3D, QVector3D, float, float, QColor>> boxes;
        for (const auto &bone: resultBones) {
            boxes.push_back(std::make_tuple(bone.headPosition, bone.tailPosition,
                bone.headRadius, bone.tailRadius, bone.color));
        }
        edgeVertices = buildBoundingBoxMeshEdges(boxes, &edgeVerticesNum);
    }
    
    if (nullptr != m_debugEdgeVertices) {
        delete[] edgeVertices;
        edgeVertices = m_debugEdgeVertices;
        m_debugEdgeVertices = nullptr;
        
        edgeVerticesNum = m_debugEdgeVerticesNum;
        m_debugEdgeVerticesNum = 0;
    }
    
    m_resultMesh = new MeshLoader(triangleVertices, triangleVerticesNum,
        edgeVertices, edgeVerticesNum);
}

void RigGenerator::generate()
{
    buildNeighborMap();
    buildBoneNodeChain();
    buildSkeleton();
    computeSkinWeights();
    buildDemoMesh();
}

void RigGenerator::process()
{
    QElapsedTimer countTimeConsumed;
    countTimeConsumed.start();
    
    generate();
    
    qDebug() << "The rig generation took" << countTimeConsumed.elapsed() << "milliseconds";
    
    this->moveToThread(QGuiApplication::instance()->thread());
    emit finished();
}
