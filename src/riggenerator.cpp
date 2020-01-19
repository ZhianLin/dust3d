#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <QGuiApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <cmath>
#include <QVector2D>
#include <queue>
#include "riggenerator.h"

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

void RigGenerator::segment()
{
    for (size_t nodeIndex = 0; nodeIndex < m_outcome->bodyNodes.size(); ++nodeIndex) {
        const auto &node = m_outcome->bodyNodes[nodeIndex];
        if (!BoneMarkIsBranchNode(node.boneMark))
            continue;
        std::unordered_set<size_t> left;
        std::unordered_set<size_t> right;
        splitByNodeIndex(nodeIndex, &left, &right);
        printf("[%s] nodeId:%s\r\n", BoneMarkToString(node.boneMark), node.nodeId.toString().toUtf8().constData());
        printf("left: ");
        for (const auto &it: left)
            printf("%lu ", it);
        printf("\r\n");
        printf("right: ");
        for (const auto &it: right)
            printf("%lu ", it);
        printf("\r\n");
    }
}

void RigGenerator::splitByNodeIndex(size_t nodeIndex,
        std::unordered_set<size_t> *left,
        std::unordered_set<size_t> *right)
{
    const auto &neighbors = m_neighborMap[nodeIndex];
    printf("neighbors.size:%lu\r\n", neighbors.size());
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

void RigGenerator::generate()
{
    buildNeighborMap();
    segment();
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
