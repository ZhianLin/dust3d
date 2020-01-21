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
            QVector3D sumOfPositions;
            for (size_t j = 0; j < source.size(); ++j) {
                sumOfPositions += m_outcome->bodyNodes[source[j]].origin;
            }
            auto middlePosition = sumOfPositions / source.size();
            std::vector<std::pair<size_t, float>> distance2Array(source.size());
            for (size_t j = 0; j < source.size(); ++j) {
                distance2Array[j] = {
                    source[j],
                    (m_outcome->bodyNodes[source[j]].origin - middlePosition).lengthSquared()
                };
            }
            (*resultNodes)[i] = std::min_element(distance2Array.begin(), distance2Array.end(), [](const std::pair<size_t, float> &first,
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
        std::vector<std::vector<size_t>> boneNodeIndices;
        std::unordered_set<size_t> visited;
        std::vector<size_t> boneNodeChain;
        collectNodesForBoneRecursively(fromNodeIndex,
            &left,
            &boneNodeIndices,
            0,
            &visited);
        removeBranchsFromNodes(&boneNodeIndices, &boneNodeChain);
        m_boneNodeChain.push_back({fromNodeIndex, boneNodeChain, isSpine});
    }
    for (size_t i = 0; i < m_boneNodeChain.size(); ++i) {
        const auto &chain = m_boneNodeChain[i];
        const auto &node = m_outcome->bodyNodes[std::get<0>(chain)];
        const auto &isSpine = std::get<2>(chain);
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
    // TODO:
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
                return m_outcome->bodyNodes[std::get<0>(m_boneNodeChain[first])].origin.y() <
                    m_outcome->bodyNodes[std::get<0>(m_boneNodeChain[second])].origin.y();
            }
            return m_outcome->bodyNodes[std::get<0>(m_boneNodeChain[first])].origin.z() <
                m_outcome->bodyNodes[std::get<0>(m_boneNodeChain[second])].origin.z();
        });
    };
    sortLimbChains(m_leftLimbChains);
    sortLimbChains(m_rightLimbChains);
    
    // TODO:
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
