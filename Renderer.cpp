#include "Renderer.h"
#include <iostream>


void GLClearError()
{
    while (glGetError() != GL_NO_ERROR);
}

bool GLLogCall(const char* function, const char* file, int line)
{
    while (GLenum error = glGetError())//this runs untill error is false
    {
        std::cout << "[OPENGL ERROR] (" << error << ")" << function <<
            " " << file << ":" << line << std::endl
            << std::endl;
        return false;
    }
    return true;
}

void Renderer::Clear() const
{
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)); // Clear the color and depth buffer
    //GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f)); // Set clear color to dark gray
	//GLCall(glClear(GL_COLOR_BUFFER_BIT)); // Clear the color buffer
}

void Renderer::Draw(const VertexArray& va, const IndexBuffer& ib, const Shader& shader) const
{
    shader.Bind();
    va.Bind();
    ib.Bind();
    GLCall(glDrawElements(GL_TRIANGLES, ib.GetCount(), GL_UNSIGNED_INT, nullptr));
    
}
