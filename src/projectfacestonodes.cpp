#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <cmath>
#include <iostream>
#include "projectfacestonodes.h"
#include "util.h"

class FacesToNodesProjector
{
public:
    FacesToNodesProjector(const std::vector<QVector3D> *vertices,
            const std::vector<std::vector<size_t>> *faces,
            const std::vector<std::pair<QVector3D, float>> *sourceNodes,
            std::vector<size_t> *faceSources) :
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
        const auto &r = nodeRadius;
        auto c = QVector3D::dotProduct(A - C, A - C) - r * r;
        if (b * b - 4 * a * c <= 0)
            return false;
        *distance2 = (faceCenter - nodePosition).lengthSquared();
        return true;
    }
    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            std::vector<std::pair<size_t, float>> distance2WithNodes;
            for (size_t j = 0; j < m_sourceNodes->size(); ++j) {
                const auto &node = (*m_sourceNodes)[j];
                float distance2 = 0.0f;
                if (!intersectionTest((*m_faces)[i], node.first, node.second, &distance2))
                    continue;
                distance2WithNodes.push_back(std::make_pair(j, distance2));
            }
            if (distance2WithNodes.empty())
                continue;
            (*m_faceSources)[i] = std::min_element(distance2WithNodes.begin(), distance2WithNodes.end(), [](const std::pair<size_t, float> &first, const std::pair<size_t, float> &second) {
                return first.second < second.second;
            })->first;
        }
    }
private:
    const std::vector<QVector3D> *m_vertices = nullptr;
    const std::vector<std::vector<size_t>> *m_faces = nullptr;
    const std::vector<std::pair<QVector3D, float>> *m_sourceNodes = nullptr;
    std::vector<size_t> *m_faceSources = nullptr;
};

class SphereParentFinder
{
public:
    SphereParentFinder(const std::vector<std::tuple<QVector3D, float, size_t>> *sourceNodesOrderedByRadius,
            std::vector<size_t> *parents) :
        m_sourceNodesOrderedByRadius(sourceNodesOrderedByRadius),
        m_parents(parents)
    {
    }
    void operator()(const tbb::blocked_range<size_t> &range) const
    {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            const auto &my = (*m_sourceNodesOrderedByRadius)[i];
            (*m_parents)[i] = i;
            for (int j = m_sourceNodesOrderedByRadius->size() - 1; j >= 0; --j) {
                const auto &potentialParent = (*m_sourceNodesOrderedByRadius)[j];
                const auto &potentialParentRadius = std::get<1>(potentialParent);
                const auto &myRadius = std::get<1>(my);
                if (myRadius <= potentialParentRadius) {
                    auto distance = (std::get<0>(my) - std::get<0>(potentialParent)).length();
                    if (distance <= myRadius + potentialParentRadius) {
                        if (potentialParentRadius - distance >= myRadius) {
                            (*m_parents)[i] = j;
                            break;
                        }
                    }
                }
            }
        }
    }
public:
    const std::vector<std::tuple<QVector3D, float, size_t>> *m_sourceNodesOrderedByRadius = nullptr;
    std::vector<size_t> *m_parents = nullptr;
};

void projectFacesToNodes(const std::vector<QVector3D> &vertices,
    const std::vector<std::vector<size_t>> &faces,
    const std::vector<std::pair<QVector3D, float>> &sourceNodes,
    std::vector<size_t> *faceSources)
{
    // Calculate the spheres which contained in bigger sphere,
    // these shperes will be replaced with the bigger shpere instead from the the source nodes
    std::vector<std::tuple<QVector3D, float, size_t>> sourceNodesOrderedByRadius;
    sourceNodesOrderedByRadius.reserve(sourceNodes.size());
    for (size_t i = 0; i < sourceNodes.size(); ++i) {
        const auto &source = sourceNodes[i];
        sourceNodesOrderedByRadius.push_back({source.first, source.second, i});
    }
    std::sort(sourceNodesOrderedByRadius.begin(), sourceNodesOrderedByRadius.end(), [](
            const std::tuple<QVector3D, float, size_t> &first,
            const std::tuple<QVector3D, float, size_t> &second) {
        return std::get<1>(first) < std::get<1>(second);
    });
    std::vector<size_t> sourceNodeParents(sourceNodesOrderedByRadius.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, sourceNodesOrderedByRadius.size()),
        SphereParentFinder(&sourceNodesOrderedByRadius, &sourceNodeParents));
    std::vector<size_t> sourceNodeAttachMap(sourceNodes.size());
    for (size_t i = 0; i < sourceNodeParents.size(); ++i) {
        sourceNodeAttachMap[std::get<2>(sourceNodesOrderedByRadius[i])] =
            std::get<2>(sourceNodesOrderedByRadius[sourceNodeParents[i]]);
    }
    
    // Resolve the faces's source nodes
    faceSources->resize(faces.size(), std::numeric_limits<size_t>::max());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, faces.size()),
        FacesToNodesProjector(&vertices, &faces, &sourceNodes, faceSources));
    
    // Replace the source node which is contained in bigger sphere
    for (size_t i = 0; i < faceSources->size(); ++i) {
        auto &source = (*faceSources)[i];
        if (source == std::numeric_limits<size_t>::max())
            continue;
        source = sourceNodeAttachMap[source];
    }
}
