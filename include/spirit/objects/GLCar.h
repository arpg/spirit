#ifndef GL_CAR_H_
#define GL_CAR_H_

#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <sophus/se3.hpp>
#include <SceneGraph/SceneGraph.h>

enum GLCarDrawType { eTriangle = 1, eMesh = 2 };

// TODO: create glwheel and glchassis files instead of glcar
class GLCar : public SceneGraph::GLMesh {
 public:
  GLCar() {
    m_sObjectName = "Car";
    m_bPerceptable = false;
  }
  // TODO: remove hardcoded mesh pathes
  void Init(GLCarDrawType eCarDrawType,
            std::string bodyMeshName = "../../../meshes/herbie.ply") {
    car_draw_type_ = eCarDrawType;
    if(bodyMeshName.empty()) {
      body_mesh_name_ = "../../../meshes/herbie.ply";
    } else {
      body_mesh_name_ = bodyMeshName;
    }
    // Set the color of the car, if we are without textures.
    chasiscolor_ = SceneGraph::GLColor(1.0f,0.4f,0.4f);
    tirecolor_ = SceneGraph::GLColor(0.2f, 0.2f, 0.2f);

    // only if the body isn't a triangle, load the meshes
    if (car_draw_type_ != eTriangle) {
      // initialize the body mesh
      const aiScene* pBodyMesh;
      // SetObjectName("mesh");
      pBodyMesh = aiImportFile(
          "/Users/saghli/code/spirit/meshes/herbie.ply",
          aiProcess_Triangulate | aiProcess_GenSmoothNormals |
              aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes |
              aiProcess_FindInvalidData | aiProcess_FixInfacingNormals);

      if (pBodyMesh == NULL) {
        throw SceneGraph::GLMeshException("Unable to load mesh.");
      } else {
        m_pScene = pBodyMesh;
      }

      GLMesh::Init(m_pScene);

      const aiScene* pWheelMesh;

      pWheelMesh =
          aiImportFile("/Users/saghli/code/spirit/meshes/wheel.ply",
                       aiProcess_Triangulate | aiProcess_GenSmoothNormals);
      if (pWheelMesh == NULL) {
        throw SceneGraph::GLMeshException("Unable to load mesh.");
      }

      // load the wheels
      wheels_vec_.resize(4);
      for (size_t i = 0; i < wheels_vec_.size(); i++) {
        wheels_vec_[i] = new GLMesh();
        wheels_vec_[i]->SetMeshColor(tirecolor_);
        wheels_vec_[i]->Init(pWheelMesh);
        // m_vWheels[i]->SetScale(m_fScale*0.75);
      }
    }
  }

  void DrawCanonicalObject() {
    if (car_draw_type_ == eTriangle) {
      SetMeshColor(chasiscolor_);
      glBegin(GL_TRIANGLES);
      //            glVertex3d(m_fScale, 0, 0);
      //            glVertex3d(-m_fScale, -m_fScale / 2, 0);
      //            glVertex3d(-m_fScale, m_fScale / 2, 0);
      glEnd();
    } else {
      // glScaled(m_dScale(0),m_dSc ale(1),m_dScale(2));
      // Set the color now if there are no textures.
//      glColor4f(color_.r, color_.g, color_.b, color_.a);
      SetMeshColor(chasiscolor_);
      GLMesh::DrawCanonicalObject();
    }
//    glColor4f(1.0, 1.0, 1.0, 1.0);
  }

  void SetRelativeWheelPose(const unsigned int& id, const Sophus::SE3d& pose) {
    if (car_draw_type_ == eMesh) {
      wheels_vec_[id]->SetPose(pose.matrix());
    }
  }

  void SetChasisColor(SceneGraph::GLColor C) { chasiscolor_ = C; }
  void SetTireColor(SceneGraph::GLColor C) { tirecolor_ = C; }

  void SetCarScale(Eigen::Vector3d scale) {
    GLMesh::SetScale(scale);
    for (size_t i = 0; i < wheels_vec_.size(); i++) {
      wheels_vec_[i]->SetScale(scale(0));
    }
  }

  std::vector<GLMesh*>& GetWheels() { return wheels_vec_; }

 protected:
  // centroid position + orientation
  Eigen::Matrix4d pose_;

  // control scale of car
  SceneGraph::GLColor chasiscolor_;
  SceneGraph::GLColor tirecolor_;
  std::string body_mesh_name_;
  std::vector<GLMesh*> wheels_vec_;
  GLCarDrawType car_draw_type_;
};

#endif  // GL_CAR_H_
