#include "Texture.h"
#include "stb_image/stb_image.h"
Texture::Texture(const std::string& path)
	: m_RendererID(0), m_FilePath(path), m_LocalBuffer(nullptr), m_Width(0), m_Height(0), m_Bpp(0)
{
	stbi_set_flip_vertically_on_load(1); // Flip the image vertically
	m_LocalBuffer = stbi_load(path.c_str(), &m_Width, &m_Height, &m_Bpp, 4); // Load the image with 4 channels (RGBA)
	//if (!m_LocalBuffer) {
	//	std::cerr << "Failed to load texture: " << path << std::endl;
	//	return;
	//}
	GLCall(glGenTextures(1, &m_RendererID));
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID));
	// Set texture parameters
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
	GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
	// Upload the texture data
	GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_Width, m_Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_LocalBuffer));
	GLCall(glBindTexture(GL_TEXTURE_2D, 0)); // Unbind the texture after uploading
	// Free the local buffer
	if (m_LocalBuffer)
	{
		stbi_image_free(m_LocalBuffer);
	}
}

Texture::~Texture()
{
	GLCall(glDeleteTextures(1, &m_RendererID));
	if (m_LocalBuffer)
	{
		stbi_image_free(m_LocalBuffer);
		m_LocalBuffer = nullptr;
	}
	m_RendererID = 0; // Reset the renderer ID
	m_Width = 0;
	m_Height = 0;
	m_Bpp = 0;
}

void Texture::Bind(unsigned int slot) const
{
	GLCall(glActiveTexture(GL_TEXTURE0 + slot)); // Activate the texture unit
	GLCall(glBindTexture(GL_TEXTURE_2D, m_RendererID)); // Bind the texture to the active texture unit
	// Note: slot is used to specify which texture unit to bind to, default is 0
	// You can bind multiple textures to different slots if needed
}

void Texture::Unbind() const
{
	GLCall(glBindTexture(GL_TEXTURE_2D, 0))
}