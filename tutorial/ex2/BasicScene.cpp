#include "BasicScene.h"
#include "igl\AABB.h"

#include "./BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/per_vertex_normals.h"



using namespace cg3d;
using namespace std;

Eigen::MatrixXi F1, F2;
Eigen::MatrixXd V1, V2;
igl::AABB<Eigen::MatrixXd, 3> tree1, tree2;
bool isMove = false;
Eigen::Vector3f velocity;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
    auto program = std::make_shared<Program>("shaders/basicShader");
    ///
    AddChild(root= Movable::Create("root"));

    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    velocity = {0.005f, 0, 0};

    /// set the scene
    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto materialF{ std::make_shared<Material>("materialF", program) }; // empty material


    /// create objects
    material->AddTexture(0, "textures/grass.bmp", 2);
    materialF->AddTexture(0, "textures/carbon.jpg", 2);

    auto Mesh{ IglLoader::MeshFromFiles("cube_igl", "data/sphere.obj") };
    ///
    auto MeshFrame { IglLoader::MeshFromFiles("cube_frame", "data/cube.off") };
    cube1 = Model::Create("cube1", Mesh, material);
    cube2 = Model::Create("cube2", Mesh, material);
    root->AddChild(cube1);
    root->AddChild(cube2);

    ///==================================================
    cubef1 = Model::Create("cubeF1", MeshFrame, materialF);
    cubef2 = Model::Create("cubeF2", MeshFrame, materialF);
    cubef1->showFaces= false;
    cubef1->showWireframe= true;
    cubef2->showFaces= false;
    cubef2->showWireframe= true;
    cube1->AddChild(cubef1);
    cube2->AddChild(cubef2);

    ///==================================================
    camera->Translate(20, Axis::Z);
    cube1->Translate({ -3, 0, 0 });
    cube2->Translate({ 3, 0, 0 });
    cube1->Scale(2);
    cube2->Scale(2);

    igl::read_triangle_mesh("data/sphere.obj", V1, F1);
    igl::read_triangle_mesh("data/sphere.obj", V2, F2);
    tree1.init(V1, F1);
    tree2.init(V2, F2);
    ///
    CubeFrame(tree1.m_box, cubef1);
    CubeFrame(tree2.m_box, cubef2);
}
///============================================================================
///cubeFrame
void BasicScene::CubeFrame(Eigen::AlignedBox<double, 3>& box, std::shared_ptr<cg3d::Model> obj) {
    Eigen::MatrixXd V(8, 3);
    Eigen::MatrixXi F(12, 3);
///Corners
    V.row(0) = box.corner(box.BottomLeftFloor);
    V.row(1)= box.corner(box.BottomRightFloor);
    V.row(2)= box.corner(box.TopLeftFloor);
    V.row(3)= box.corner(box.TopRightFloor);
    V.row(4)= box.corner(box.BottomLeftCeil);
    V.row(5)= box.corner(box.BottomRightCeil);
    V.row(6)= box.corner(box.TopLeftCeil);
    V.row(7)= box.corner(box.TopRightCeil);

///Triangles
    F.row(0) = Eigen::Vector3i(0, 1, 3);
    F.row(1) = Eigen::Vector3i(0, 2, 3);
    F.row(2) = Eigen::Vector3i(1, 5, 7);
    F.row(3) = Eigen::Vector3i(1, 3, 7);
    F.row(4) = Eigen::Vector3i(0, 2, 6);
    F.row(5) = Eigen::Vector3i(0, 4, 6);
    F.row(6) = Eigen::Vector3i(4, 6, 7);
    F.row(7) = Eigen::Vector3i(4, 5, 7);
    F.row(8) = Eigen::Vector3i(0, 4, 5);
    F.row(9) = Eigen::Vector3i(0, 1, 5);
    F.row(10) = Eigen::Vector3i(2, 6, 7);
    F.row(11) = Eigen::Vector3i(2, 3, 7);


    Eigen::MatrixXd normals = Eigen::MatrixXd();
    igl::per_vertex_normals(V, F, normals);
    Eigen::MatrixXd texCoords = Eigen::MatrixXd::Zero(V.rows(), 2);

    auto mesh = obj->GetMeshList();
    mesh[0]->data.push_back({ V, F, normals, texCoords });
    obj->SetMeshList(mesh);
    obj->meshIndex = obj->meshIndex+1;
}

bool BasicScene::isBoxCollided(std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2,
                               Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2) {

    Eigen::Matrix3f A = model1->GetRotation();
    Eigen::Matrix3f B = model2->GetRotation();

    //Eigen::Vector3d Pa = box1.center();
    Eigen::Vector3d Ax = A.col(0).cast<double>();
    Eigen::Vector3d Ay = A.col(1).cast<double>();
    Eigen::Vector3d Az = A.col(2).cast<double>();
    double Wa = box1.sizes()(0);
    double Ha = box1.sizes()(1);
    double Da = box1.sizes()(2);

    //Eigen::Vector3d Pb = box2.center();
    Eigen::Vector3d Bx = B.col(0).cast<double>();
    Eigen::Vector3d By = B.col(1).cast<double>();
    Eigen::Vector3d Bz = B.col(2).cast<double>();
    double Wb = box2.sizes()(0);
    double Hb = box2.sizes()(1);
    double Db = box2.sizes()(2);

    //Eigen::Vector3d T = Pa - Pb;
    Eigen::Vector4d BoxA_Center = Eigen::Vector4d(box1.center()[0], box1.center()[1], box1.center()[2], 1);
    Eigen::Vector4d BoxB_Center = Eigen::Vector4d(box2.center()[0], box2.center()[1], box2.center()[2], 1);
    Eigen::Vector4d Distance4Dim = model2->GetTransform().cast<double>() * BoxB_Center - model1->GetTransform().cast<double>() * BoxA_Center;
    Eigen::Vector3d T = Distance4Dim.head(3);


    // if one of this is true then the boxes do not collide - return false, otherwise return true
    // Ax
    if (abs(T.dot(Ax)) > Wa + abs(Wb * Ax.dot(Bx)) + abs(Hb * Ax.dot(By)) + abs(Db * Ax.dot(Bz))) return false;
    // Ay
    if (abs(T.dot(Ay)) > Ha + abs(Wb * Ay.dot(Bx)) + abs(Hb * Ay.dot(By)) + abs(Db * Ay.dot(Bz))) return false;
    // Az
    if (abs(T.dot(Az)) > Da + abs(Wb * Az.dot(Bx)) + abs(Hb * Az.dot(By)) + abs(Db * Az.dot(Bz))) return false;
    // Bx
    if (abs(T.dot(Bx)) > Wb + abs(Wa * Ax.dot(Bx)) + abs(Ha * Ay.dot(Bx)) + abs(Da * Az.dot(Bx))) return false;
    // By
    if (abs(T.dot(By)) > Hb + abs(Wa * Ax.dot(By)) + abs(Ha * Ay.dot(By)) + abs(Da * Az.dot(By))) return false;
    // Bz
    if (abs(T.dot(Bz)) > Db + abs(Wa * Ax.dot(Bz)) + abs(Ha * Ay.dot(Bz)) + abs(Da * Az.dot(Bz))) return false;
    // Ax x Bx
    if (abs((T.dot(Az) * Ay.dot(Bx) - (T.dot(Ay) * Az.dot(Bx)))) >
        abs(Ha * Az.dot(Bx)) + abs(Da * Ay.dot(Bx)) + abs(Hb * Ax.dot(Bz)) + abs(Db * Ax.dot(By))) return false;
    // Ax x By
    if (abs((T.dot(Az) * Ay.dot(By) - (T.dot(Ay) * Az.dot(By)))) >
        abs(Ha * Az.dot(By)) + abs(Da * Ay.dot(By)) + abs(Wb * Ax.dot(Bz)) + abs(Db * Ax.dot(Bx))) return false;
    // Ax x Bz
    if (abs((T.dot(Az) * Ay.dot(Bz) - (T.dot(Ay) * Az.dot(Bz)))) >
        abs(Ha * Az.dot(Bz)) + abs(Da * Ay.dot(Bz)) + abs(Wb * Ax.dot(By)) + abs(Hb * Ax.dot(Bx))) return false;
    // Ay x Bx
    if (abs((T.dot(Ax) * Az.dot(Bx) - (T.dot(Az) * Ax.dot(Bx)))) >
        abs(Wa * Az.dot(Bx)) + abs(Da * Ax.dot(Bx)) + abs(Hb * Ay.dot(Bz)) + abs(Db * Ay.dot(By))) return false;
    // Ay x By
    if (abs((T.dot(Ax) * Az.dot(By) - (T.dot(Az) * Ax.dot(By)))) >
        abs(Wa * Az.dot(By)) + abs(Da * Ax.dot(By)) + abs(Wb * Ay.dot(Bz)) + abs(Db * Ay.dot(Bx))) return false;
    // Ay x Bz
    if (abs((T.dot(Ax) * Az.dot(Bz) - (T.dot(Az) * Ax.dot(Bz)))) >
        abs(Wa * Az.dot(Bz)) + abs(Da * Ax.dot(Bz)) + abs(Wb * Ay.dot(By)) + abs(Hb * Ay.dot(Bx))) return false;
    // Az x Bx
    if (abs((T.dot(Ay) * Ax.dot(Bx) - (T.dot(Ax) * Ay.dot(Bx)))) >
        abs(Wa * Ay.dot(Bx)) + abs(Ha * Ax.dot(Bx)) + abs(Hb * Az.dot(Bz)) + abs(Db * Az.dot(By))) return false;
    // Az x By
    if (abs((T.dot(Ay) * Ax.dot(By) - (T.dot(Ax) * Ay.dot(By)))) >
        abs(Wa * Ay.dot(By)) + abs(Ha * Ax.dot(By)) + abs(Wb * Az.dot(Bz)) + abs(Db * Az.dot(Bx))) return false;
    // Az x Bz
    if (abs((T.dot(Ay) * Ax.dot(Bz) - (T.dot(Ax) * Ay.dot(Bz)))) >
        abs(Wa * Ay.dot(Bz)) + abs(Ha * Ax.dot(Bz)) + abs(Wb * Az.dot(By)) + abs(Hb * Az.dot(Bx))) return false;

    return true;
}

bool BasicScene::checkForCollisionBetweenObjects(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2)
{
    if (tree1 == nullptr || tree2 == nullptr) return false;

    if (!isBoxCollided(cube1, cube2, tree1->m_box, tree2->m_box)) {
        // no collision, return false
        return false;
    }
    
    // collision happend...
    // check if boxes are leaves
    if (tree1->is_leaf() && tree2->is_leaf()) {
        // done, draw smallest boxes
        isMove = false;
        CubeFrame(tree1->m_box, cubef1);
        CubeFrame(tree2->m_box, cubef2);
        return true;
    }
    else if (tree1->is_leaf()) {
        // tree2 is not leaf
        return checkForCollisionBetweenObjects(tree1, tree2->m_left) || checkForCollisionBetweenObjects(tree1, tree2->m_right);
    }
    else if (tree2->is_leaf()) {
        // tree1 is not leaf
        return checkForCollisionBetweenObjects(tree1->m_left, tree2) || checkForCollisionBetweenObjects(tree1->m_right, tree2);
    }

    // check rest options
    return checkForCollisionBetweenObjects(tree1->m_left, tree2->m_right) || checkForCollisionBetweenObjects(tree1->m_right, tree2->m_left)
        || checkForCollisionBetweenObjects(tree1->m_left, tree2->m_left) || checkForCollisionBetweenObjects(tree1->m_right, tree2->m_right);
}

///============================================================================
void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);

    if (isMove) {
        checkForCollisionBetweenObjects(&tree1, &tree2);
        cube1->Translate(velocity);
    }

    //cube->Rotate(0.01f, Axis::XYZ);
}

void BasicScene::KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_RIGHT:
            velocity = {0.005f, 0, 0};
            break;
        case GLFW_KEY_LEFT:
            velocity = {-0.005f, 0, 0};
            break;
        case GLFW_KEY_UP:
            velocity = {0, 0.005f, 0};
            break;
        case GLFW_KEY_DOWN:
            velocity = {0, -0.005f, 0};
            break;
        case GLFW_KEY_SPACE:
            isMove = !isMove;
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        }
    }
}

