#include <fstream>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include "reactphysics3d.h"

extern "C" {
#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
}

using namespace std;

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 800;

char WINDOW_NAME[] = "Cosmic Marble!";
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

GLuint marbleShaderProgram;
GLuint groundShaderProgram;

void setup_shaders() {
    // marble vertex shader
    const char * vertex_shader_code =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aNormal;\n"
        "out vec3 FragPos;\n"
        "out vec3 Normal;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "void main() {\n"
        "  FragPos = aPos;\n"
        "  Normal = mat3(transpose(inverse(model))) * aNormal;\n"
        "  gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
        "}";
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, & vertex_shader_code, NULL);
    glCompileShader(vertexShader);
    int success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("marble vertex shader");
    }

    // ground vertex shader
    const char * ground_vertex_shader_code =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "out vec3 FragPos;\n"
        "uniform mat4 projection;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "void main() {\n"
        "  FragPos = aPos;\n"
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

    // marble fragment shader
    ifstream shaderfile("marbletest.glsl");
    shaderfile.seekg(0, ios::end);
    int length = shaderfile.tellg();
    shaderfile.seekg(0, ios::beg);
    char * fragment_shader_code = new char[length+1];
    shaderfile.read(fragment_shader_code, length);
    fragment_shader_code[length] = '\0';
    shaderfile.close();

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

    // shader programs
    marbleShaderProgram = glCreateProgram();
    glAttachShader(marbleShaderProgram, vertexShader);
    glAttachShader(marbleShaderProgram, fragmentShader);
    glLinkProgram(marbleShaderProgram);
    glGetProgramiv(marbleShaderProgram, GL_LINK_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(marbleShaderProgram, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("marble shader program");
    }

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

    // delete shaders (unneeded after program link)
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(ground_fragmentShader);
}

void add_point(vector<float> & values, glm::vec3 point) {
    values.push_back(point.x);
    values.push_back(point.y);
    values.push_back(point.z);
}

float min_height = 0.0;
float max_height = 1.0;
// column-major order
const int GH_HEIGHT = 7;
const int GH_WIDTH = 7;
float ground_heights[GH_HEIGHT][GH_WIDTH] = {
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
};

rp3d::DynamicsWorld * world;
rp3d::RigidBody * ground_body;
rp3d::RigidBody * marble_body;

GLuint ground_vao;
int ground_nverts;

GLuint marble_vao;

void setup_scene() {
    // gravity and world
    rp3d::Vector3 gravity(0.0, -9.81, 0.0);
    world = new rp3d::DynamicsWorld(gravity);

    // ground heightfield
    rp3d::Transform ground_pose(rp3d::Vector3(0, 0.5, 0), rp3d::Quaternion::identity());
    ground_body = world->createRigidBody(ground_pose);
    ground_body->setType(rp3d::BodyType::STATIC);
    auto ground_shape = new rp3d::HeightFieldShape(7, 7, min_height, max_height, ground_heights, rp3d::HeightFieldShape::HeightDataType::HEIGHT_FLOAT_TYPE);
    ground_body->addCollisionShape(ground_shape, rp3d::Transform(), 1.0);
    auto ground_mat = ground_body->getMaterial();
    ground_mat.setBounciness(rp3d::decimal(0.1));
    ground_mat.setFrictionCoefficient(rp3d::decimal(0.0001));

    vector<float> ground_vertices = {};
    vector<int> ground_elements = {};
    // loop over ground_heights, processing a quad at a time
    for (int ghy=0 ; ghy<GH_HEIGHT-1 ; ghy+=1) {
        float y = float(ghy) - float(GH_HEIGHT/2);
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

    // marble sphere
    rp3d::Transform marble_pose(rp3d::Vector3(-0.5, 3.5, -1.0), rp3d::Quaternion::identity());
    marble_body = world->createRigidBody(marble_pose);
    auto marble_shape = new rp3d::SphereShape(0.5);
    marble_body->addCollisionShape(marble_shape, rp3d::Transform(), 1.0);
    auto marble_mat = marble_body->getMaterial();
    marble_mat.setBounciness(rp3d::decimal(0.1));
    marble_mat.setFrictionCoefficient(rp3d::decimal(0.0001));

    float vertices[] = {
        0.0,0.0,0.0, 0.0,0.0,1.0,
        1.0,0.0,0.0, 0.0,0.0,1.0,
        1.0,1.0,0.0, 0.0,0.0,1.0,
        0.0,1.0,0.0, 0.0,0.0,1.0
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
}

int frame = 0;

float time_step = 1.0 / 1000.0;
void physics_step(float dt) {
    for (float n=0 ; n<dt ; n+=time_step) {
        world->update(time_step);
    }
}

void draw_scene() {
    // camera view pose
    auto view = glm::lookAt(glm::vec3(0,3,-5), glm::vec3(0,0,0), glm::vec3(0,1,0));

    // background color
    glClearColor(0.2, 0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ground heightfield
    // set up uniform values
    glUseProgram(groundShaderProgram);
    unsigned int g_projectionLoc = glGetUniformLocation(groundShaderProgram, "projection");
    unsigned int g_viewLoc = glGetUniformLocation(groundShaderProgram, "view");
    unsigned int g_lightPosLoc = glGetUniformLocation(groundShaderProgram, "lightPos");
    unsigned int g_lightColorLoc = glGetUniformLocation(groundShaderProgram, "lightColor");

    auto g_projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);
    glUniformMatrix4fv(g_projectionLoc, 1, GL_FALSE, glm::value_ptr(g_projection));

    glUniformMatrix4fv(g_viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glUniform3f(g_lightPosLoc, 1.0, 1.0, -1.0);
    glUniform3f(g_lightColorLoc, 1.0, 1.0, 1.0);

    // draw
    auto g_model = glm::mat4(1.0f);
    unsigned int g_modelLoc = glGetUniformLocation(groundShaderProgram, "model");
    glUniformMatrix4fv(g_modelLoc, 1, GL_FALSE, glm::value_ptr(g_model));
    unsigned int g_objectColorLoc = glGetUniformLocation(groundShaderProgram, "objectColor");
    glm::vec3 g_color = {0.0, 1.0, 1.0};
    glUniform3fv(g_objectColorLoc, 1, glm::value_ptr(g_color));

    glBindVertexArray(ground_vao);
    glDrawElements(GL_TRIANGLES, ground_nverts, GL_UNSIGNED_INT, 0);

    // marble sphere
    // set up uniform values
    glUseProgram(marbleShaderProgram);
    unsigned int projectionLoc = glGetUniformLocation(marbleShaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(marbleShaderProgram, "view");
    unsigned int lightPosLoc = glGetUniformLocation(marbleShaderProgram, "lightPos");
    unsigned int lightColorLoc = glGetUniformLocation(marbleShaderProgram, "lightColor");

    auto projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glUniform3f(lightPosLoc, 1.0, 1.0, -1.0);
    glUniform3f(lightColorLoc, 1.0, 1.0, 1.0);

    // draw
    auto model = glm::mat4(1.0f);
    auto marble_tran = marble_body->getTransform();

    rp3d::Vector3 tempv = marble_tran.getPosition();
    glm::vec3 marble_pos;
    marble_pos.x = tempv.x;
    marble_pos.y = tempv.y;
    marble_pos.z = tempv.z;
    //cout << marble_pos.x << ',' << marble_pos.y << ',' << marble_pos.z << endl;
    model = glm::translate(model, marble_pos);
    model = glm::translate(model, glm::vec3(-0.5, -0.5, 0));
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

    unsigned int objectColorLoc = glGetUniformLocation(marbleShaderProgram, "objectColor");
    glm::vec3 color = {0.0, 1.0, 1.0};
    glUniform3fv(objectColorLoc, 1, glm::value_ptr(color));

    unsigned int iTimeLoc = glGetUniformLocation(marbleShaderProgram, "iTime");
    glUniform1f(iTimeLoc, frame * (20.0 / 1000.0));

    glBindVertexArray(marble_vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

unsigned int FRAME_TICK;

uint32_t timer_callback(uint32_t interval, void * param) {
    SDL_Event e;
    e.type = FRAME_TICK;
    SDL_PushEvent(& e);

    return interval;
}

int main(int nargs, char * args[])
{
    init();

    setup_shaders();

    setup_scene();

    // timer tick every 20msec
    FRAME_TICK = SDL_RegisterEvents(1);
    SDL_AddTimer(20, timer_callback, NULL);

    bool done = false;
    while (! done)
    {
        SDL_Event e;
        SDL_WaitEvent(& e); //TODO check for error

        if (e.type == SDL_QUIT) done = true;
        else if (e.type == FRAME_TICK) {
            physics_step(20.0/1000.0); // step forward 20msec

            draw_scene();
            SDL_GL_SwapWindow(gWindow);

            frame += 1;
            SDL_FlushEvent(FRAME_TICK); // don't pile up frame ticks
        }
    }

    close();

    return 0;
}
