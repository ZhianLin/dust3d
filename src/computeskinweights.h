#ifndef DUST3D_COMPUTE_SKIN_WEIGHTS_H
#define DUST3D_COMPUTE_SKIN_WEIGHTS_H
#include <QVector3D>
#include <vector>
#include "rigger.h"

bool computeSkinWeights(const std::vector<QVector3D> &vertices,
        const std::vector<std::vector<size_t>> &faces,
        const std::vector<RiggerBone> &bones,
        std::map<int, RiggerVertexWeights> *vertexWeights);

#endif
