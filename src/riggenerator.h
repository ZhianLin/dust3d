#ifndef DUST3D_RIG_GENERATOR_H
#define DUST3D_RIG_GENERATOR_H
#include <QObject>
#include <QThread>
#include <QDebug>
#include <unordered_set>
#include "outcome.h"
#include "meshloader.h"
#include "rigger.h"
#include "rigtype.h"

class RigGenerator : public QObject
{
    Q_OBJECT
public:
    RigGenerator(RigType rigType, const Outcome &outcome);
    ~RigGenerator();
    MeshLoader *takeResultMesh();
    std::vector<RiggerBone> *takeResultBones();
    std::map<int, RiggerVertexWeights> *takeResultWeights();
    const std::vector<std::pair<QtMsgType, QString>> &messages();
    Outcome *takeOutcome();
    bool isSucceed();
    void generate();
signals:
    void finished();
public slots:
    void process();
private:
    RigType m_rigType = RigType::None;
    Outcome *m_outcome = nullptr;
    MeshLoader *m_resultMesh = nullptr;
    std::vector<RiggerBone> *m_resultBones = nullptr;
    std::map<int, RiggerVertexWeights> *m_resultWeights = nullptr;
    std::vector<std::pair<QtMsgType, QString>> m_messages;
    std::map<size_t, std::unordered_set<size_t>> m_neighborMap;
    std::vector<std::tuple<size_t, std::vector<size_t>, bool>> m_boneNodeChain;
    std::vector<size_t> m_neckChains;
    std::vector<size_t> m_leftLimbChains;
    std::vector<size_t> m_rightLimbChains;
    std::vector<size_t> m_tailChains;
    std::vector<size_t> m_spineChains;
    bool m_isSpineVertical = false;
    bool m_isSucceed = false;
    void buildNeighborMap();
    void buildBoneNodeChain();
    void buildSkeleton();
    void calculateSpineDirection(bool *isVertical);
    void splitByNodeIndex(size_t nodeIndex,
        std::unordered_set<size_t> *left,
        std::unordered_set<size_t> *right);
    void collectNodes(size_t fromNodeIndex,
        std::unordered_set<size_t> *container,
        std::unordered_set<size_t> *visited);
    void collectNodesForBoneRecursively(size_t fromNodeIndex,
        const std::unordered_set<size_t> *limitedNodeIndices,
        std::vector<std::vector<size_t>> *boneNodeIndices,
        size_t depth,
        std::unordered_set<size_t> *visited);
    void removeBranchsFromNodes(const std::vector<std::vector<size_t>> *boneNodeIndices,
        std::vector<size_t> *resultNodes);
};

#endif
