#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <QDebug>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "computeskinweights.h"
#include "util.h"

class FacesToNodesProjector
{
public:
    FacesToNodesProjector(const std::vector<QVector3D> *vertices,
            const std::vector<std::vector<size_t>> *faces,
            const std::vector<std::tuple<QVector3D, float, size_t, float>> *sourceNodes,
            std::vector<std::vector<std::pair<size_t, float>>> *faceSources) :
        m_vertices(vertices),
        m_faces(faces),
        m_sourceNodes(sourceNodes),
        m_faceSources(faceSources)
    {
    }
    bool intersectionTest(const std::vector<size_t> &face,
            const QVector3D &nodePosition, float nodeRadius,
            float *distance2) const
    {
        QVector3D faceCenter;
        for (const auto &it: face) {
            faceCenter += (*m_vertices)[it];
        }
        if (face.size() > 0)
            faceCenter /= face.size();
        const auto &A = faceCenter;
        const auto &C = nodePosition;
        auto B = -polygonNormal(*m_vertices, face);
        auto a = QVector3D::dotProduct(B, B);
        auto b = 2.0f * QVector3D::dotProduct(B, A - C);
        if (b >= 0.0)
            return false;
        const auto r2 = nodeRadius * nodeRadius;
        auto c = QVector3D::dotProduct(A - C, A - C) - r2;
        if (b * b - 4 * a * c <= 0)
            return false;
        *distance2 = (faceCenter - nodePosition).lengthSquared();
        if (std::sqrt(*distance2) > nodeRadius + nodeRadius)
            return false;
        return true;
    }
    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            std::vector<std::pair<size_t, float>> distance2WithNodes;
            for (size_t j = 0; j < m_sourceNodes->size(); ++j) {
                const auto &node = (*m_sourceNodes)[j];
                if (0 == std::get<2>(node))
                    continue;
                float distance2 = 0.0f;
                if (!intersectionTest((*m_faces)[i], std::get<0>(node), std::get<1>(node), &distance2))
                    continue;
                distance2WithNodes.push_back(std::make_pair(j, distance2));
            }
            if (distance2WithNodes.empty())
                continue;
            std::sort(distance2WithNodes.begin(), distance2WithNodes.end(), [](const std::pair<size_t, float> &first,
                    const std::pair<size_t, float> &second) {
                return first.second < second.second;
            });
            (*m_faceSources)[i] = distance2WithNodes;
        }
    }
private:
    const std::vector<QVector3D> *m_vertices = nullptr;
    const std::vector<std::vector<size_t>> *m_faces = nullptr;
    const std::vector<std::tuple<QVector3D, float, size_t, float>> *m_sourceNodes = nullptr;
    std::vector<std::vector<std::pair<size_t, float>>> *m_faceSources = nullptr;
};

static void buildInterpolatedNodes(const std::vector<RiggerBone> &bones,
        std::vector<std::tuple<QVector3D, float, size_t, float>> *targetNodes)
{
    std::vector<std::pair<size_t, size_t>> boneEdges;
    std::map<size_t, size_t> oldToNewMap;
    auto addNode = [&](size_t boneIndex) {
        auto findNew = oldToNewMap.find(boneIndex);
        if (findNew != oldToNewMap.end())
            return findNew->second;
        auto newIndex = targetNodes->size();
        targetNodes->push_back({bones[boneIndex].headPosition,
            bones[boneIndex].headRadius,
            boneIndex,
            1.0f
        });
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
        //printf("processing:%s\r\n", bones[boneIndex].name.toUtf8().constData());
        waitQueue.pop();
        const auto &bone = bones[boneIndex];
        if (bone.children.empty()) {
            targetNodes->push_back({bone.tailPosition,
                bone.tailRadius,
                boneIndex,
                1.0f
            });
            continue;
        }
        for (const auto &childIndex: bone.children) {
            addEdge(boneIndex, childIndex);
            waitQueue.push(childIndex);
        }
    }
    
    for (const auto &it: boneEdges) {
        const auto &fromNode = (*targetNodes)[it.first];
        const auto &toNode = (*targetNodes)[it.second];
        const auto &fromRadius = std::get<1>(fromNode);
        const auto &toRadius = std::get<1>(toNode);
        const auto &fromIndex = std::get<2>(fromNode);
        const auto &toIndex = std::get<2>(toNode);
        if (0 == fromIndex)
            continue;
        const auto &fromPosition = std::get<0>(fromNode);
        const auto &toPosition = std::get<0>(toNode);
        float length = (fromPosition - toPosition).length();
        float segments = length / 0.02;
        if (qFuzzyIsNull(segments))
            continue;
        if (segments > 100)
            segments = 100;
        float segmentLength = 1.0f / segments;
        float offset = segmentLength;
        while (offset < 1.0f) {
            float radius = fromRadius * (1.0f - offset) + toRadius * offset;
            targetNodes->push_back(std::make_tuple(
                fromPosition * (1.0f - offset) + toPosition * offset,
                radius,
                offset <= 0.5 ? fromIndex : toIndex,
                1.0f - offset
            ));
            offset += segmentLength;
        }
    }
}

bool computeSkinWeights(const std::vector<QVector3D> &vertices,
        const std::vector<std::vector<size_t>> &faces,
        const std::vector<RiggerBone> &bones,
        std::map<int, RiggerVertexWeights> *vertexWeights)
{
    std::vector<std::tuple<QVector3D, float, size_t, float>> interpolatedNodes;
    buildInterpolatedNodes(bones, &interpolatedNodes);
    std::vector<std::vector<std::pair<size_t, float>>> faceSources(faces.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, faces.size()),
        FacesToNodesProjector(&vertices, &faces, &interpolatedNodes, &faceSources));
    //for (size_t i = 0; i < interpolatedNodes.size(); ++i) {
    //    const auto &bone = bones[std::get<2>(interpolatedNodes[i])];
    //    std::cout << "interpolated[" << i << "]:" << bone.name.toUtf8().constData() << " " << std::get<3>(interpolatedNodes[i]) << std::endl;
    //}
    std::map<size_t, std::unordered_set<size_t>> vertexSourceIndices;
    for (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const auto &source = faceSources[faceIndex];
        for (const auto &vertexIndex: faces[faceIndex]) {
            auto &target = vertexSourceIndices[vertexIndex];
            for (const auto &sourceIt: source)
                target.insert(sourceIt.first);
        }
    }
    for (const auto &it: vertexSourceIndices) {
        const auto &vertexIndex = it.first;
        auto &target = (*vertexWeights)[vertexIndex];
        for (const auto &nodeIndex: it.second) {
            const auto &boneIndex = std::get<2>(interpolatedNodes[nodeIndex]);
            const auto &bone = bones[boneIndex];
            target.addBone(boneIndex, (vertices[vertexIndex] - bone.headPosition).length());
        }
        target.finalizeWeights();
    }
    
    //for (const auto &it: *vertexWeights) {
    //    std::cout << "vertex:" << it.first << "|";
    //    for (size_t i = 0; i < 4; ++i) {
    //        std::cout << it.second.boneIndices[i] << ":" << it.second.boneWeights[i] << " ";
    //    }
    //    std::cout << std::endl;
    //}
    
    return true;
}
