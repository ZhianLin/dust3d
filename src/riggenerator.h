#ifndef DUST3D_RIG_GENERATOR_H
#define DUST3D_RIG_GENERATOR_H
#include <QObject>
#include <QThread>
#include <QDebug>
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
    bool m_isSucceed = false;
};

#endif
