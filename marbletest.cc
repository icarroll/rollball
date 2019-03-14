#include <iostream>
#include <fstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

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

GLuint shaderProgram;

void setup_shaders() {
    // vertex shader
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
        die("vertex shader");
    }

    // fragment shader
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

    // shader program
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, & success);
    if (! success) {
        char infoLog[512];
        glGetShaderInfoLog(shaderProgram, 512, NULL, infoLog);
        cout << infoLog << endl;
        die("shader program");
    }

    // delete shaders (unneeded after program link)
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

int frame = 0;

void draw_scene() {
    // background color
    glClearColor(0.2, 0.3, 0.3, 1.0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set up uniform values
    glUseProgram(shaderProgram);
    unsigned int projectionLoc = glGetUniformLocation(shaderProgram, "projection");
    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    unsigned int lightPosLoc = glGetUniformLocation(shaderProgram, "lightPos");
    unsigned int lightColorLoc = glGetUniformLocation(shaderProgram, "lightColor");

    auto projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 100.0f);
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    auto view = glm::mat4(1.0f);
    view = glm::translate(view, glm::vec3(0.0, 0.0, -4.0));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glUniform3f(lightPosLoc, 1.0, 1.0, -1.0);
    glUniform3f(lightColorLoc, 1.0, 1.0, 1.0);

    // draw quad
    float vertices[] = {
        0.0,0.0,0.0, 0.0,0.0,1.0,
        1.0,0.0,0.0, 0.0,0.0,1.0,
        1.0,1.0,0.0, 0.0,0.0,1.0,
        0.0,1.0,0.0, 0.0,0.0,1.0
    };
    GLuint vao;
    glGenVertexArrays(1, & vao);
    GLuint vbo;
    glGenBuffers(1, & vbo);
    glBindVertexArray(vao);
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

    unsigned int modelLoc = glGetUniformLocation(shaderProgram, "model");
    auto model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-3, -3, 0));
    model = glm::scale(model, glm::vec3(6, 6, 6));
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

    unsigned int objectColorLoc = glGetUniformLocation(shaderProgram, "objectColor");
    glm::vec3 color = {0.0, 1.0, 1.0};
    glUniform3fv(objectColorLoc, 1, glm::value_ptr(color));

    unsigned int iTimeLoc = glGetUniformLocation(shaderProgram, "iTime");
    glUniform1f(iTimeLoc, frame * (20.0 / 1000.0));

    glBindVertexArray(vao);

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

    //setup_scene();

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
            draw_scene();
            SDL_GL_SwapWindow(gWindow);
            frame += 1;
            SDL_FlushEvent(FRAME_TICK); // don't pile up frame ticks
        }
    }

    close();

    return 0;
}
