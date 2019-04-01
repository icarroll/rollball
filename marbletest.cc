#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>

#include "reactphysics3d.h"
#include "collision/ContactManifold.h"
#include "constraint/ContactPoint.h"

extern "C" {
#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
}

using namespace std;

const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 800;
const float ASPECT = float(SCREEN_WIDTH) / float(SCREEN_HEIGHT);

char WINDOW_NAME[] = "Lost Your Marbles?";
SDL_Window * gWindow = NULL;
SDL_GLContext gContext;

void die(string message) {
    cout << message << endl;
    exit(1);
}

// print out OpenGL error messages
void GLAPIENTRY MessageCallback(
        GLenum source,
        GLenum type,
        GLuint id,
        GLenum severity,
        GLsizei length,
        const GLchar * message,
        const void * userParam) {
    if (type==GL_DEBUG_TYPE_ERROR) cerr << "GL CALLBACK: " << (type==GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "") << " type=" << type << " severity=" << severity << " message=" << message << endl;
}

void init() {
    // init SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) die("SDL");
    if (! SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1")) die("texture");

    // init SDL GL
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);

    gWindow = SDL_CreateWindow(WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED,
                               SCREEN_WIDTH, SCREEN_HEIGHT,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (gWindow == NULL) die("window");

    //memset(& gContext, 0, sizeof(gContext));
    gContext = SDL_GL_CreateContext(gWindow);
    if (! gContext) die("gl context");

    // init GLEW
    glewExperimental = GL_TRUE; 
    GLenum glewError = glewInit();
    if (glewError != GLEW_OK) die("glew");
    if (! GLEW_VERSION_2_1) die("glew2.1");

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(MessageCallback, 0);

    // GL viewport
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
}

void close()
{
    //TODO close OpenGL

    SDL_DestroyWindow(gWindow);
    gWindow = NULL;

    SDL_Quit();
}

glm::vec3 xlat(rp3d::Vector3 invec) {
    glm::vec3 outvec;
    outvec.x = invec.x;
    outvec.y = invec.y;
    outvec.z = invec.z;
    return outvec;
}

float oriented_angle(rp3d::Vector2 rp_v1, rp3d::Vector2 rp_v2) {
    glm::vec2 v1 = glm::normalize(glm::vec2(rp_v1.x, rp_v1.y));
    glm::vec2 v2 = glm::normalize(glm::vec2(rp_v2.x, rp_v2.y));
    return glm::orientedAngle(v1, v2);
}

GLuint marbleShaderProgram;
GLuint groundShaderProgram;

void setup_shaders() {
    /*
    const char * vertex_shader_code =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aNormal;\n"
        "out vec3 FragPos;\n"
        "out vec3 Normal;\n"
        "uniform vec3 camera_pos;\n"
        "uniform vec3 up_vector;\n"
        "uniform vec3 obj_pos;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "mat4 rotationMatrix(vec3 axis, float angle)\n"
        "{\n"
        "  axis = normalize(axis);\n"
        "  float s = sin(angle);\n"
        "  float c = cos(angle);\n"
        "  float oc = 1.0 - c;\n"
        "\n"
        "  return mat4(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,\n"
        "              oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,\n"
        "              oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,\n"
        "              0.0,                                0.0,                                0.0,                                1.0);\n"
        "}\n"
        "void main() {\n"
        "  FragPos = aPos + vec3(0.5,0.5,0);\n"
        "  Normal = mat3(transpose(inverse(model))) * aNormal;\n"

        "  vec3 look = normalize(obj_pos - camera_pos);\n"
        "  vec3 rot_axis = cross(look, normalize(aNormal));\n"
        "  float rot_angle = asin(length(rot_axis));\n"
        "  if (dot(look, normalize(aNormal)) < 0) {\n"
        "    rot_angle = -rot_angle;\n"
        "    FragPos.x = 1-FragPos.x;\n"
        "  }\n"
        "  mat4 billboard = rotationMatrix(rot_axis, rot_angle);\n"
        "  gl_Position = projection * view * model * billboard * vec4(aPos, 1.0);\n"
        "}";
    */
    // marble vertex shader
    ifstream bbvertfile("marbletest_billboard_vert.glsl");
    bbvertfile.seekg(0, ios::end);
    int length = bbvertfile.tellg();
    bbvertfile.seekg(0, ios::beg);
    char * bbvertcode = new char[length+1];
    bbvertfile.read(bbvertcode, length);
    bbvertcode[length] = '\0';
    bbvertfile.close();

    unsigned int bbvertShader;
    bbvertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(bbvertShader, 1, & bbvertcode, NULL);
    glCompileShader(bbvertShader);
    int success;
    glGetShaderiv(bbvertShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(bbvertShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("billboard vertex shader");
    }

    // marble geometry shader
    ifstream bbgeomfile("marbletest_billboard_geom.glsl");
    bbgeomfile.seekg(0, ios::end);
    length = bbgeomfile.tellg();
    bbgeomfile.seekg(0, ios::beg);
    char * bbgeomcode = new char[length+1];
    bbgeomfile.read(bbgeomcode, length);
    bbgeomcode[length] = '\0';
    bbgeomfile.close();

    unsigned int bbgeomShader;
    bbgeomShader = glCreateShader(GL_GEOMETRY_SHADER);
    glShaderSource(bbgeomShader, 1, & bbgeomcode, NULL);
    glCompileShader(bbgeomShader);
    glGetShaderiv(bbgeomShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(bbgeomShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("billboard geometry shader");
    }

    // marble fragment shader
    ifstream shaderfile("marbletest.glsl");
    shaderfile.seekg(0, ios::end);
    length = shaderfile.tellg();
    shaderfile.seekg(0, ios::beg);
    char * fragment_shader_code = new char[length+1];
    shaderfile.read(fragment_shader_code, length);
    fragment_shader_code[length] = '\0';
    shaderfile.close();
    /*
    const char * fragment_shader_code = 
        "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main() {\n"
        "  FragColor = vec4(1,1,1,1);\n"
        "}";
    */

    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, & fragment_shader_code, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("fragment shader");
    }

    // marble shader program
    marbleShaderProgram = glCreateProgram();
    glAttachShader(marbleShaderProgram, bbvertShader);
    glAttachShader(marbleShaderProgram, bbgeomShader);
    glAttachShader(marbleShaderProgram, fragmentShader);
    glLinkProgram(marbleShaderProgram);
    glGetProgramiv(marbleShaderProgram, GL_LINK_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(marbleShaderProgram, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("marble shader program");
    }

    // ground vertex shader
    const char * ground_vertex_shader_code =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "out vec3 thingPos;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "void main() {\n"
        "  thingPos = aPos;\n"
        "  gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
        "}";
    unsigned int ground_vertexShader;
    ground_vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(ground_vertexShader, 1, & ground_vertex_shader_code, NULL);
    glCompileShader(ground_vertexShader);
    glGetShaderiv(ground_vertexShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(ground_vertexShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("ground vertex shader");
    }

    // ground geometry shader
    ifstream ground_geom_shaderfile("marbletest_ground_geom.glsl");
    ground_geom_shaderfile.seekg(0, ios::end);
    length = ground_geom_shaderfile.tellg();
    ground_geom_shaderfile.seekg(0, ios::beg);
    char * ground_geometry_shader_code = new char[length+1];
    ground_geom_shaderfile.read(ground_geometry_shader_code, length);
    ground_geometry_shader_code[length] = '\0';
    ground_geom_shaderfile.close();

    unsigned int ground_geometryShader;
    ground_geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
    glShaderSource(ground_geometryShader, 1, & ground_geometry_shader_code, NULL);
    glCompileShader(ground_geometryShader);
    glGetShaderiv(ground_geometryShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(ground_geometryShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("ground geometry shader");
    }

    // ground fragment shader
    ifstream ground_shaderfile("marbletest_ground.glsl");
    ground_shaderfile.seekg(0, ios::end);
    length = ground_shaderfile.tellg();
    ground_shaderfile.seekg(0, ios::beg);
    char * ground_fragment_shader_code = new char[length+1];
    ground_shaderfile.read(ground_fragment_shader_code, length);
    ground_fragment_shader_code[length] = '\0';
    ground_shaderfile.close();

    unsigned int ground_fragmentShader;
    ground_fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(ground_fragmentShader, 1, & ground_fragment_shader_code, NULL);
    glCompileShader(ground_fragmentShader);
    glGetShaderiv(ground_fragmentShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(ground_fragmentShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("ground fragment shader");
    }

    // ground shader program
    groundShaderProgram = glCreateProgram();
    glAttachShader(groundShaderProgram, ground_vertexShader);
    glAttachShader(groundShaderProgram, ground_geometryShader);
    glAttachShader(groundShaderProgram, ground_fragmentShader);
    glLinkProgram(groundShaderProgram);
    glGetProgramiv(groundShaderProgram, GL_LINK_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(groundShaderProgram, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("ground shader program");
    }

    //TODO delete shaders (unneeded after program link)
    /*
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(ground_fragmentShader);
    */
}

void add_point(vector<float> & values, glm::vec3 point) {
    values.push_back(point.x);
    values.push_back(point.y);
    values.push_back(point.z);
}

float min_height = 0.0;
float max_height = 1.0;
// column-major order
const int GH_LENGTH = 8;
const int GH_WIDTH = 13;
float ground_heights[GH_LENGTH][GH_WIDTH] = {
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.5, 1.0, 0.5, 0.2, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
};

rp3d::DynamicsWorld * world;
rp3d::RigidBody * ground_body;
rp3d::RigidBody * marble_body;
rp3d::RigidBody * camera_body;

GLuint ground_vao;
int ground_nverts;

GLuint marble_vao;

rp3d::Vector3 gravity(0.0, -9.81, 0.0);

rp3d::Vector3 SPAWN_POS(0.0, 3.5, -0.1);

vector<float> ground_vertices = {};
vector<int> ground_elements = {};
rp3d::TriangleMesh ground_tri_mesh;

void setup_scene() {
    // gravity and world
    world = new rp3d::DynamicsWorld(gravity);

    // camera
    rp3d::Transform camera_pose(rp3d::Vector3(0, 3, 5), rp3d::Quaternion::identity());
    camera_body = world->createRigidBody(camera_pose);
    camera_body->setType(rp3d::BodyType::KINEMATIC);

    // ground heightfield
    rp3d::Transform ground_pose(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());
    ground_body = world->createRigidBody(ground_pose);
    ground_body->setType(rp3d::BodyType::STATIC);
    auto ground_mat = ground_body->getMaterial();
    ground_mat.setBounciness(rp3d::decimal(0.5));
    ground_mat.setFrictionCoefficient(rp3d::decimal(0.5));
    ground_body->setMaterial(ground_mat);

    // loop over ground_heights, processing a quad at a time
    for (int ghy=0 ; ghy<GH_LENGTH-1 ; ghy+=1) {
        float y = float(ghy) - float(GH_LENGTH/2);
        float cy = y + 0.5;
        for (int ghx=0 ; ghx<GH_WIDTH-1 ; ghx+=1) {
            float x = float(ghx) - float(GH_WIDTH/2);
            float cx = x + 0.5;

            float ch = 0.0;
            for (int ix=0 ; ix<2 ; ix+=1) {
                for (int iy=0 ; iy<2 ; iy+=1) {
                    ch += ground_heights[ghy + iy][ghx + ix];
                }
            }
            ch /= 4.0;

            int elem = ground_vertices.size() / 3;

            add_point(ground_vertices, glm::vec3(cx, ch, cy));
            add_point(ground_vertices, glm::vec3(x, ground_heights[ghy][ghx], y));
            add_point(ground_vertices, glm::vec3(x+1, ground_heights[ghy][ghx+1], y));
            add_point(ground_vertices, glm::vec3(x+1, ground_heights[ghy+1][ghx+1], y+1));
            add_point(ground_vertices, glm::vec3(x, ground_heights[ghy+1][ghx], y+1));

            for (int n=1 ; n<=3 ; n+=1) {
                ground_elements.push_back(elem);
                ground_elements.push_back(elem+n);
                ground_elements.push_back(elem+n+1);
            }
            ground_elements.push_back(elem);
            ground_elements.push_back(elem+4);
            ground_elements.push_back(elem+1);
        }
    }
    glGenVertexArrays(1, & ground_vao);
    GLuint gvbo;
    glGenBuffers(1, & gvbo);
    glBindVertexArray(ground_vao);
    glBindBuffer(GL_ARRAY_BUFFER, gvbo);
    glBufferData(GL_ARRAY_BUFFER, ground_vertices.size() * sizeof(float), & ground_vertices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    GLuint gebo;
    glGenBuffers(1, & gebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gebo);
    ground_nverts = ground_elements.size();
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ground_nverts * sizeof(float), & ground_elements[0], GL_STATIC_DRAW);

    int nverts = ground_vertices.size() / 3;
    int ntris = ground_elements.size() / 3;
    rp3d::TriangleVertexArray * tva = new rp3d::TriangleVertexArray(nverts, & ground_vertices[0], 3*sizeof(float), ntris, & ground_elements[0], 3 * sizeof(int), rp3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE, rp3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
    ground_tri_mesh.addSubpart(tva);
    rp3d::ConcaveMeshShape * ground_shape = new rp3d::ConcaveMeshShape(& ground_tri_mesh);
    ground_body->addCollisionShape(ground_shape, rp3d::Transform(), 1.0);

    // marble sphere
    //rp3d::Transform marble_pose(rp3d::Vector3(-0.5, 3.5, -1.0), rp3d::Quaternion::identity());
    rp3d::Transform marble_pose(SPAWN_POS, rp3d::Quaternion::identity());
    marble_body = world->createRigidBody(marble_pose);
    auto marble_shape = new rp3d::SphereShape(0.5);
    marble_body->addCollisionShape(marble_shape, rp3d::Transform(), 1.0);
    auto marble_mat = marble_body->getMaterial();
    marble_mat.setBounciness(rp3d::decimal(0.5));
    marble_mat.setFrictionCoefficient(rp3d::decimal(0.5));
    marble_body->setMaterial(marble_mat);

    float points[] = {0.0f, 0.0f};
    GLuint vbo;
    glGenBuffers(1, & vbo);
    glGenVertexArrays(1, & marble_vao);
    glBindVertexArray(marble_vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), nullptr);

    /*
    float vertices[] = {
        -0.5, -0.5, 0.0, 0.0,0.0,1.0,
         0.5, -0.5, 0.0, 0.0,0.0,1.0,
         0.5,  0.5, 0.0, 0.0,0.0,1.0,
        -0.5,  0.5, 0.0, 0.0,0.0,1.0
    };
    glGenVertexArrays(1, & marble_vao);
    GLuint vbo;
    glGenBuffers(1, & vbo);
    glBindVertexArray(marble_vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void *) (3*sizeof(float)));
    glEnableVertexAttribArray(1);

    unsigned int elements[] = {
        0, 1, 2,
        2, 3, 0
    };
    GLuint ebo;
    glGenBuffers(1, & ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);
    */
}

const float DIST = 6.0;
const float HEIGHT = 3.0;
const float FOLLOW = 1.0;
const float LIFT = 2.0;
const float TURN = 2.0;
const float JUMP = 200.0;
const float OUT = 10.0;
const float ORBIT = 5.0;

set<SDL_Keycode> keys_down;

int frame = 0;

float time_step = 1.0 / 1000.0;
void physics_step(float dt) {
    // process keys
    bool stop = false;
    rp3d::Vector3 mar_torque(0,0,0);
    if (keys_down.count(SDLK_x)) stop = true;
    else {
        if (keys_down.count(SDLK_w)) mar_torque += rp3d::Vector3(-1,0,0);
        if (keys_down.count(SDLK_a)) mar_torque += rp3d::Vector3(0,0,1);
        if (keys_down.count(SDLK_s)) mar_torque += rp3d::Vector3(1,0,0);
        if (keys_down.count(SDLK_d)) mar_torque += rp3d::Vector3(0,0,-1);
        mar_torque = mar_torque.getUnit() * 3.0;
    }

    // check for jump
    rp3d::Vector3 mar_force(0,0,0);
    if (keys_down.count(SDLK_SPACE)) {
        const rp3d::ContactManifoldListElement * ce =
                marble_body->getContactManifoldsList();
        auto e = const_cast<rp3d::ContactManifoldListElement *>(ce);
        for ( ; e!=nullptr ; e=e->getNext()) {
            rp3d::ContactManifold * m = e->getContactManifold();
            rp3d::ContactPoint * p = m->getContactPoints();
            for ( ; p!=nullptr ; p=p->getNext()) {
                mar_force += p->getNormal();
            }
        }

        if (mar_force.length() > 0.0) mar_force = mar_force.getUnit() * JUMP;
    }

    // orbit camera
    float cam_strafe_right = 0.0;
    if (keys_down.count(SDLK_q)) cam_strafe_right -= ORBIT;
    if (keys_down.count(SDLK_e)) cam_strafe_right += ORBIT;

    // physics update loop
    for (float n=0 ; n<dt ; n+=time_step) {
        if (stop) marble_body->setAngularVelocity(rp3d::Vector3(0,0,0));
        else marble_body->applyTorque(mar_torque);
        marble_body->applyForceToCenterOfMass(mar_force);

        // move camera
        rp3d::Vector3 rp_mpos = marble_body->getTransform().getPosition();
        auto camera_tran = camera_body->getTransform();
        rp3d::Vector3 rp_cpos = camera_tran.getPosition();
        rp3d::Quaternion rp_cornt = camera_tran.getOrientation();

        rp3d::Vector3 rp_clook = rp_cornt * rp3d::Vector3(0,0,-1);
        rp3d::Vector3 rp_cmpos = rp_mpos - rp_cpos;
        rp3d::Vector2 rp_clook_xz(rp_clook.x, rp_clook.z);
        rp3d::Vector2 rp_cmpos_xz(rp_cmpos.x, rp_cmpos.z);

        rp3d::Vector2 pln_vel = rp_clook_xz * (rp_cmpos_xz.length() - DIST) * FOLLOW;
        //rp3d::Quaternion right90 = rp3d::Quaternion::fromEulerAngles(0,M_PI/2.0,0);
        //rp3d::Vector2 rp_cright_xz = right90 * rp_clook_xz;
        rp3d::Vector2 rp_cright_xz = rp_clook_xz.getOneUnitOrthogonalVector();
        pln_vel += rp_cright_xz * cam_strafe_right;
        float up_vel = (rp_mpos.y - rp_cpos.y + HEIGHT) * LIFT;
        camera_body->setLinearVelocity(rp3d::Vector3(pln_vel.x,up_vel,pln_vel.y));

        float rot_vel = oriented_angle(rp_cmpos_xz, rp_clook_xz) * TURN;
        camera_body->setAngularVelocity(rp3d::Vector3(0,rot_vel,0));

        // physics step
        world->update(time_step);
    }

    // check for fall out
    rp3d::Transform t = marble_body->getTransform();
    rp3d::Vector3 pos = t.getPosition();
    if (pos.y < -OUT || abs(pos.x) > OUT || abs(pos.y) > OUT) {
        t.setPosition(SPAWN_POS);
        marble_body->setTransform(t);
        marble_body->setLinearVelocity(rp3d::Vector3(0,0,0));
    }
}

void draw_scene() {
    // get marble position
    auto marble_tran = marble_body->getTransform();
    glm::vec3 marble_pos = xlat(marble_tran.getPosition());

    // camera setup
    auto projection = glm::perspective(glm::radians(90.0f), ASPECT, 0.1f, 100.0f);
    //auto projection = glm::ortho(-4.0f, 4.0f, -4.0f, 4.0f, 0.1f, 100.0f);

    //auto camera_pos = glm::vec3(0,3,-5);
    //glm::vec3 camera_pos = glm::rotateY(glm::vec3(0,3,-5), glm::radians((float) frame/5.0f));
    //glm::vec3 camera_pos = glm::rotateY(glm::vec3(0,3,-5), glm::radians(0.0f + cos(glm::radians((float) frame))));
    //glm::vec3 camera_pos = glm::rotateY(glm::vec3(0,3,5), glm::radians(5.0f));
    auto camera_tran = camera_body->getTransform();
    rp3d::Vector3 rp_cpos = camera_tran.getPosition();
    glm::vec3 camera_pos = xlat(rp_cpos);
    rp3d::Quaternion rp_cornt = camera_tran.getOrientation();
    glm::vec3 camera_tgt = xlat(rp_cornt * rp3d::Vector3(0,0,-1) + rp_cpos);
    //auto camera_tgt = glm::vec3(0,0,0);
    //auto camera_tgt = marble_pos;
    auto up_vector = glm::vec3(0,1,0);
    auto view = glm::lookAt(camera_pos, camera_tgt, up_vector);

    auto camera_look = glm::normalize(camera_pos - camera_tgt);
    auto camera_right = glm::normalize(glm::cross(camera_look, up_vector));
    auto camera_up = glm::normalize(glm::cross(camera_right, camera_look));

    /*
    auto temp = glm::vec4(0,0,-1, 1);
    temp = view * temp;
    cout << temp.x << ',' << temp.y << ',' << temp.z << ':' << temp.w << endl;
    */

    // light setup
    //float ang = frame / 50.0 * M_PI;
    //auto light_pos = glm::vec3(10.0 * cos(ang), 10.0, 10.0 * sin(ang));
    auto light_pos = glm::vec3(0.0, 3.0, 0.0);
    auto light_color = glm::vec3(0.9, 0.9, 0.9);

    // background color
    glClearColor(0.2, 0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // marble sphere
    // set up uniform values
    glUseProgram(marbleShaderProgram);
    unsigned int projectionLoc = glGetUniformLocation(marbleShaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(marbleShaderProgram, "view");
    //unsigned int lightPosLoc = glGetUniformLocation(marbleShaderProgram, "lightPos");
    //unsigned int lightColorLoc = glGetUniformLocation(marbleShaderProgram, "lightColor");

    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    //glUniform3f(lightPosLoc, 1.0, 1.0, -1.0);
    //glUniform3fv(lightPosLoc, 1, glm::value_ptr(light_pos));
    //glUniform3fv(lightColorLoc, 1, glm::value_ptr(light_color));

    unsigned int cameraPosLoc = glGetUniformLocation(marbleShaderProgram, "cameraPos");
    glUniform3fv(cameraPosLoc, 1, glm::value_ptr(camera_pos));

    // draw marble
    unsigned int camera_posLoc = glGetUniformLocation(marbleShaderProgram, "camera_pos");
    glUniform3f(camera_posLoc, camera_pos.x, camera_pos.y, camera_pos.z);
    unsigned int up_vectorLoc = glGetUniformLocation(marbleShaderProgram, "up_vector");
    glUniform3f(up_vectorLoc, up_vector.x, up_vector.y, up_vector.z);

    //cout << marble_pos.x << ',' << marble_pos.y << ',' << marble_pos.z << endl;
    unsigned int obj_posLoc = glGetUniformLocation(marbleShaderProgram, "obj_pos");
    glUniform3fv(obj_posLoc, 1, glm::value_ptr(marble_pos));

    unsigned int camera_lookLoc = glGetUniformLocation(marbleShaderProgram, "camera_look");
    glUniform3fv(camera_lookLoc, 1, glm::value_ptr(camera_look));
    unsigned int camera_rightLoc = glGetUniformLocation(marbleShaderProgram, "camera_right");
    glUniform3fv(camera_rightLoc, 1, glm::value_ptr(camera_right));
    unsigned int camera_upLoc = glGetUniformLocation(marbleShaderProgram, "camera_up");
    glUniform3fv(camera_upLoc, 1, glm::value_ptr(camera_up));

    unsigned int aspectLoc = glGetUniformLocation(marbleShaderProgram, "aspect");
    glUniform1f(aspectLoc, ASPECT);

    auto model = glm::mat4(1.0f);
    model = glm::translate(model, marble_pos);
    unsigned int modelLoc = glGetUniformLocation(marbleShaderProgram, "model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

    rp3d::Matrix3x3 tempm = marble_tran.getOrientation().getMatrix();
    glm::mat3 marble_rot;
    for (int y=0 ; y<3 ; y+=1) {
        for (int x=0 ; x<3 ; x+=1) {
            marble_rot[y][x] = tempm[y][x];
        }
    }
    unsigned int obj_rotationLoc = glGetUniformLocation(marbleShaderProgram, "obj_rotation");
    glUniformMatrix3fv(obj_rotationLoc, 1, GL_FALSE, glm::value_ptr(marble_rot));

    // color is ignored for this fragment shader
    //unsigned int objectColorLoc = glGetUniformLocation(marbleShaderProgram, "objectColor");
    //glm::vec3 color = {0.0, 1.0, 1.0};
    //glUniform3fv(objectColorLoc, 1, glm::value_ptr(color));

    /*
    unsigned int iTimeLoc = glGetUniformLocation(marbleShaderProgram, "iTime");
    glUniform1f(iTimeLoc, frame * (20.0 / 1000.0));
    */

    glBindVertexArray(marble_vao);
    //glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_POINTS, 0, 1);

    // ground heightfield
    // set up uniform values
    glUseProgram(groundShaderProgram);
    unsigned int g_projectionLoc = glGetUniformLocation(groundShaderProgram, "projection");
    unsigned int g_viewLoc = glGetUniformLocation(groundShaderProgram, "view");
    unsigned int g_lightPosLoc = glGetUniformLocation(groundShaderProgram, "lightPos");
    unsigned int g_lightColorLoc = glGetUniformLocation(groundShaderProgram, "lightColor");

    glUniformMatrix4fv(g_projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glUniformMatrix4fv(g_viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glUniform3fv(g_lightPosLoc, 1, glm::value_ptr(light_pos));
    glUniform3fv(g_lightColorLoc, 1, glm::value_ptr(light_color));

    unsigned int sphere_posLoc = glGetUniformLocation(groundShaderProgram, "sphere_pos");
    glUniform3fv(sphere_posLoc, 1, glm::value_ptr(marble_pos));
    unsigned int sphere_rLoc = glGetUniformLocation(groundShaderProgram, "sphere_r");
    glUniform1f(sphere_rLoc, 0.5);

    // draw ground
    auto g_model = glm::mat4(1.0f);
    unsigned int g_modelLoc = glGetUniformLocation(groundShaderProgram, "model");
    glUniformMatrix4fv(g_modelLoc, 1, GL_FALSE, glm::value_ptr(g_model));
    unsigned int g_objectColorLoc = glGetUniformLocation(groundShaderProgram, "objectColor");
    glm::vec3 g_color = {0.0, 1.0, 1.0};
    glUniform3fv(g_objectColorLoc, 1, glm::value_ptr(g_color));

    glBindVertexArray(ground_vao);
    glDrawElements(GL_TRIANGLES, ground_nverts, GL_UNSIGNED_INT, 0);

    // "veil" for sphere depth value testing
    /*
    glUseProgram(veilShaderProgram);
    unsigned int v_projectionLoc = glGetUniformLocation(veilShaderProgram, "projection");
    unsigned int v_viewLoc = glGetUniformLocation(veilShaderProgram, "view");
    */
}

unsigned int FRAME_TICK;

uint32_t timer_callback(uint32_t interval, void * param) {
    SDL_Event e;
    e.type = FRAME_TICK;
    SDL_PushEvent(& e);

    return interval;
}

int main(int nargs, char * args[]) {
    init();

    setup_shaders();

    setup_scene();

    // timer tick every 20msec
    FRAME_TICK = SDL_RegisterEvents(1);
    SDL_AddTimer(20, timer_callback, NULL);

    bool go = false;
    bool done = false;
    while (! done)
    {
        SDL_Event e;
        SDL_WaitEvent(& e); //TODO check for error

        if (e.type == SDL_QUIT) done = true;
        else if (e.type == FRAME_TICK) {
            if (go) physics_step(20.0/1000.0); // step forward 20msec

            draw_scene();
            SDL_GL_SwapWindow(gWindow);

            frame += 1;
            SDL_FlushEvent(FRAME_TICK); // don't pile up frame ticks
        }
        else if (e.type == SDL_KEYDOWN) {
            if (e.key.keysym.sym == SDLK_SPACE) go = true;
            else if (e.key.keysym.sym == SDLK_ESCAPE) done = true;

            keys_down.insert(e.key.keysym.sym);
        }
	else if (e.type == SDL_KEYUP) keys_down.erase(e.key.keysym.sym);
    }

    close();

    return 0;
}
