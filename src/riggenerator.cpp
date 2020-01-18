#include <QGuiApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <cmath>
#include <QVector2D>
#include "riggenerator.h"

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

void RigGenerator::generate()
{
    if (nullptr == m_outcome->triangleSourceNodes())
        return;
    
    const auto &inputVerticesPositions = m_outcome->vertices;
    const auto &triangleSourceNodes = *m_outcome->triangleSourceNodes();
    const std::vector<std::vector<QVector3D>> *triangleVertexNormals = m_outcome->triangleVertexNormals();
    const std::vector<QVector3D> *triangleTangents = m_outcome->triangleTangents();
    
    // TODO:
    
    /*
    if (m_isSucceed) {
        qDebug() << "Rig succeed";
    } else {
        qDebug() << "Rig failed";
    }
    if (nullptr != m_autoRigger) {
        m_messages = m_autoRigger->messages();
        for (const auto &message: m_autoRigger->messages()) {
            qDebug() << "errorType:" << message.first << "Message:" << message.second;
        }
    }
    
    // Blend vertices colors according to bone weights
    
    std::vector<QColor> inputVerticesColors(m_outcome->vertices.size(), Qt::black);
    if (m_isSucceed) {
        const auto &resultWeights = m_autoRigger->resultWeights();
        const auto &resultBones = m_autoRigger->resultBones();
        
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
    
    m_resultMesh = new MeshLoader(triangleVertices, triangleVerticesNum,
        edgeVertices, edgeVerticesNum);
    */
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
