#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <QGuiApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <cmath>
#include <QVector2D>
#include <queue>
#include "riggenerator.h"
#include "util.h"
#include "computeskinweights.h"

class PartEndpointsStitcher
{
public:
    PartEndpointsStitcher(const std::vector<OutcomeNode> *nodes,
            const std::vector<std::pair<QUuid, size_t>> *partEndpoints,
            std::vector<std::pair<std::pair<QUuid, size_t>, float>> *stitchResult) :
        m_nodes(nodes),
        m_partEndpoints(partEndpoints),
        m_stitchResult(stitchResult)
    {
    }
    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            std::vector<std::pair<std::pair<QUuid, size_t>, float>> distance2WithNodes;
            const auto &endpoint = (*m_partEndpoints)[i];
            const auto &endpointNode = (*m_nodes)[endpoint.second];
            for (size_t j = 0; j < m_nodes->size(); ++j) {
                const auto &node = (*m_nodes)[j];
                if (node.partId == endpoint.first ||
                        node.mirroredByPartId == endpoint.first ||
                        node.mirrorFromPartId == endpoint.first) {
                    continue;
                }
                distance2WithNodes.push_back({{node.partId, j},
                    (endpointNode.origin - node.origin).lengthSquared()});
            }
            if (distance2WithNodes.empty())
                continue;
            (*m_stitchResult)[i] = *std::min_element(distance2WithNodes.begin(), distance2WithNodes.end(), [](const std::pair<std::pair<QUuid, size_t>, float> &first, const std::pair<std::pair<QUuid, size_t>, float> &second) {
                return first.second < second.second;
            });
        }
    }
private:
    const std::vector<OutcomeNode> *m_nodes = nullptr;
    const std::vector<std::pair<QUuid, size_t>> *m_partEndpoints = nullptr;
    std::vector<std::pair<std::pair<QUuid, size_t>, float>> *m_stitchResult = nullptr;
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
    
    std::vector<std::pair<QUuid, size_t>> partEndpoints;
    for (const auto &it: m_neighborMap) {
        if (it.second.size() >= 2) {
            continue;
        }
        const auto &node = m_outcome->bodyNodes[it.first];
        partEndpoints.push_back({node.partId, it.first});
    }
    
    std::vector<std::pair<std::pair<QUuid, size_t>, float>> stitchResult(partEndpoints.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, partEndpoints.size()),
        PartEndpointsStitcher(&m_outcome->bodyNodes, &partEndpoints,
            &stitchResult));
    
    for (size_t i = 0; i < stitchResult.size(); ++i) {
        const auto &endpoint = partEndpoints[i];
        const auto &neighbors = m_neighborMap[endpoint.second];
        float distance2WithNeighbor = std::numeric_limits<float>::max();
        if (!neighbors.empty()) {
            distance2WithNeighbor = (m_outcome->bodyNodes[endpoint.second].origin - m_outcome->bodyNodes[*neighbors.begin()].origin).lengthSquared();
        }
        const auto &result = stitchResult[i];
        if (result.second >= distance2WithNeighbor)
            continue;
        auto fromNodeIndex = nodeIdToIndexMap[{endpoint.first, m_outcome->bodyNodes[endpoint.second].nodeId}];
        auto toNodeIndex = nodeIdToIndexMap[{result.first.first, m_outcome->bodyNodes[result.first.second].nodeId}];
        m_neighborMap[fromNodeIndex].insert(toNodeIndex);
        m_neighborMap[toNodeIndex].insert(fromNodeIndex);
    }
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
        if (i < 2) {
            std::vector<std::pair<size_t, float>> radisArray(source.size());
            for (size_t j = 0; j < source.size(); ++j) {
                radisArray[j] = {
                    source[j],
                    m_outcome->bodyNodes[source[j]].radius
                };
            }
            (*resultNodes)[i] = std::max_element(radisArray.begin(), radisArray.end(), [](const std::pair<size_t, float> &first,
                        const std::pair<size_t, float> &second) {
                    return first.second < second.second;
                })->first;
            continue;
        }
        QVector3D lastDirection = (m_outcome->bodyNodes[(*resultNodes)[i - 1]].origin -
            m_outcome->bodyNodes[(*resultNodes)[i - 2]].origin).normalized();
        const auto &lastPosition = m_outcome->bodyNodes[(*resultNodes)[i - 1]].origin;
        std::vector<std::pair<size_t, float>> anglesArray(source.size());
        for (size_t j = 0; j < source.size(); ++j) {
            auto direction = (m_outcome->bodyNodes[source[j]].origin - lastPosition).normalized();
            anglesArray[j] = {
                source[j],
                radianBetweenVectors(lastDirection, direction)
            };
        }
        (*resultNodes)[i] = std::min_element(anglesArray.begin(), anglesArray.end(), [](const std::pair<size_t, float> &first,
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
        isJointFlags.resize(boneNodeChain.size(), false);
        for (size_t i = 0; i < boneNodeIndices.size(); ++i) {
            for (const auto &nodeIndex: boneNodeIndices[i]) {
                if (BoneMark::None == m_outcome->bodyNodes[nodeIndex].boneMark)
                    continue;
                isJointFlags[i] = true;
                break;
            }
        }
        m_boneNodeChain.push_back({fromNodeIndex, boneNodeChain, isSpine, isJointFlags, attachNodeIndex});
    }
    for (size_t i = 0; i < m_boneNodeChain.size(); ++i) {
        const auto &chain = m_boneNodeChain[i];
        const auto &node = m_outcome->bodyNodes[chain.fromNodeIndex];
        const auto &isSpine = chain.isSpine;
        //printf("Chain[%lu] %s %s", i, BoneMarkToString(node.boneMark), isSpine ? "SPINE " : "");
        //printf("|");
        //for (size_t j = 0; j < chain.nodeIndices.size(); ++j) {
        //    printf("%lu%s ", chain.nodeIndices[j], chain.nodeIsJointFlags[j] ? "(JOINT)" : "");
        //}
        //printf("\r\n");
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
        const auto &nodeIndices = m_boneNodeChain[boneNodeChainIndex].nodeIndices;
        distance2WithSpine.reserve(nodeIndices.size());
        for (size_t j = 0; j < nodeIndices.size(); ++j) {
            const auto &nodeIndex = nodeIndices[j];
            distance2WithSpine.push_back({
                j,
                (m_outcome->bodyNodes[nodeIndex].origin - limbMiddle).lengthSquared()
            });
        }
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
    
    std::map<QString, int> boneNameToIndexMap;
    size_t rootJointIndex = m_attachLimbsToSpineJointIndices[0];
    
    m_resultBones = new std::vector<RiggerBone>;
    m_resultWeights = new std::map<int, RiggerVertexWeights>;
    
    {
        const auto &firstSpineNode = m_outcome->bodyNodes[m_spineJoints[rootJointIndex]];
        RiggerBone bone;
        bone.headPosition = QVector3D(0.0, 0.0, 0.0);
        bone.tailPosition = firstSpineNode.origin;
        bone.name = QString("Body");
        bone.index = m_resultBones->size();
        boneNameToIndexMap.insert({bone.name, (int)bone.index});
        m_resultBones->push_back(bone);
    }
    
    auto attachedBoneIndex = [&](size_t spineJointIndex) {
        if (spineJointIndex == rootJointIndex) {
            return boneNameToIndexMap[QString("Body")];
        }
        return boneNameToIndexMap[QString("Spine") + QString::number(spineJointIndex - rootJointIndex)];
    };
    
    for (size_t spineJointIndex = rootJointIndex;
            spineJointIndex + 1 < m_spineJoints.size();
            ++spineJointIndex) {
        const auto &currentNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex]];
        const auto &nextNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex + 1]];
        RiggerBone bone;
        bone.headPosition = currentNode.origin;
        bone.tailPosition = nextNode.origin;
        bone.name = QString("Spine") + QString::number(spineJointIndex + 1 - rootJointIndex);
        bone.index = m_resultBones->size();
        boneNameToIndexMap.insert({bone.name, (int)bone.index});
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
        bone.name = QString("Virtual_") + (*m_resultBones)[parentIndex].name + QString("_") + chainName;
        bone.index = m_resultBones->size();
        boneNameToIndexMap.insert({bone.name, (int)bone.index});
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
            bone.name = chainName + QString("_Joint") + QString::number(limbJointIndex + 1);
            bone.index = m_resultBones->size();
            boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (limbJointIndex > 0) {
                auto parentName = chainName + QString("_Joint") + QString::number(limbJointIndex);
                (*m_resultBones)[boneNameToIndexMap[parentName]].children.push_back(bone.index);
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
            bone.name = QString("Neck_Joint") + QString::number(neckJointIndex + 1);
            bone.index = m_resultBones->size();
            boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (neckJointIndex > 0) {
                auto parentName = QString("Neck_Joint") + QString::number(neckJointIndex);
                (*m_resultBones)[boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    }
    
    if (!m_tailJoints.empty()) {
        {
            const auto &spineJointIndex = rootJointIndex;
            const auto &spineNode = m_outcome->bodyNodes[m_spineJoints[spineJointIndex]];
            const auto &tailFirstNode = m_outcome->bodyNodes[m_tailJoints[0]];
            RiggerBone bone;
            bone.headPosition = spineNode.origin;
            bone.tailPosition = tailFirstNode.origin;
            bone.name = QString("Virtual_Body_Tail");
            bone.index = m_resultBones->size();
            boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
        }
    
        for (size_t tailJointIndex = 0;
                tailJointIndex + 1 < m_tailJoints.size();
                ++tailJointIndex) {
            const auto &currentNode = m_outcome->bodyNodes[m_tailJoints[tailJointIndex]];
            const auto &nextNode = m_outcome->bodyNodes[m_tailJoints[tailJointIndex + 1]];
            RiggerBone bone;
            bone.headPosition = currentNode.origin;
            bone.tailPosition = nextNode.origin;
            bone.name = QString("Tail_Joint") + QString::number(tailJointIndex + 1);
            bone.index = m_resultBones->size();
            boneNameToIndexMap.insert({bone.name, (int)bone.index});
            m_resultBones->push_back(bone);
            if (tailJointIndex > 0) {
                auto parentName = QString("Tail_Joint") + QString::number(tailJointIndex);
                (*m_resultBones)[boneNameToIndexMap[parentName]].children.push_back(bone.index);
            }
        }
    }
    
    if (!m_resultBones->empty()) {
        std::vector<QVector3D> boneNodes;
        std::vector<std::pair<size_t, size_t>> boneEdges;
        std::map<size_t, size_t> oldToNewMap;
        auto addNode = [&](size_t boneIndex) {
            auto findNew = oldToNewMap.find(boneIndex);
            if (findNew != oldToNewMap.end())
                return findNew->second;
            auto newIndex = boneNodes.size();
            boneNodes.push_back((*m_resultBones)[boneIndex].headPosition);
            oldToNewMap.insert({boneIndex, newIndex});
            return newIndex;
        };
        auto addEdge = [&](size_t fromBoneIndex, size_t toBoneIndex) {
            auto fromNewIndex = addNode(fromBoneIndex);
            auto toNewIndex = addNode(toBoneIndex);
            boneEdges.push_back({fromNewIndex, toNewIndex});
        };

        std::queue<int> waitQueue;
        waitQueue.push(0);
        while (!waitQueue.empty()) {
            auto boneIndex = (int)waitQueue.front();
            waitQueue.pop();
            const auto &bone = (*m_resultBones)[boneIndex];
            for (const auto &childIndex: bone.children) {
                addEdge(boneIndex, childIndex);
                waitQueue.push(childIndex);
            }
        }
        
        for (size_t i = 0; i < boneNodes.size(); ++i) {
            printf("node[%lu]\r\n", i);
        }
        for (size_t i = 0; i < boneEdges.size(); ++i) {
            printf("edge[%lu]: %lu %lu\r\n", i, boneEdges[i].first, boneEdges[i].second);
        }
        
        computeSkinWeights(m_outcome->vertices, m_outcome->triangles, boneNodes, boneEdges);
    }
    
    m_isSucceed = true;
}

void RigGenerator::extractBranchJoints()
{
    auto extractJoints = [&](const BoneNoeChain &chain, std::vector<size_t> *joints) {
        joints->push_back(chain.fromNodeIndex);
        for (size_t i = 0; i < chain.nodeIndices.size(); ++i) {
            if (chain.nodeIsJointFlags[i] || i + 1 == chain.nodeIndices.size())
                joints->push_back(chain.nodeIndices[i]);
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
        std::reverse(spine.nodeIndices.begin(), spine.nodeIndices.end());
        std::reverse(spine.nodeIsJointFlags.begin(), spine.nodeIsJointFlags.end());
        for (auto &it: m_attachLimbsToSpineChainPositions) {
            it = spine.nodeIndices.size() - 1 - it;
        }
    }
    m_attachLimbsToSpineJointIndices.resize(m_attachLimbsToSpineChainPositions.size());
    for (size_t i = 0; i < spine.nodeIndices.size(); ++i) {
        bool limbsAttached = false;
        for (size_t j = 0; j < m_attachLimbsToSpineChainPositions.size(); ++j) {
            if (i == m_attachLimbsToSpineChainPositions[j]) {
                m_attachLimbsToSpineJointIndices[j] = m_spineJoints.size();
                limbsAttached = true;
            }
        }
        if (limbsAttached || spine.nodeIsJointFlags[i]) {
            m_spineJoints.push_back(spine.nodeIndices[i]);
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

void RigGenerator::generate()
{
    buildNeighborMap();
    buildBoneNodeChain();
    buildSkeleton();
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
