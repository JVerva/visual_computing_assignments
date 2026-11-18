#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "OBJ_Loader.h"
#include "camera.hpp"
#include "geometry.hpp"
#include "shader.hpp"
#include "texture_manager.hpp"
#include "themes.hpp"
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <glad/glad.h>
#include <glm/detail/qualifier.hpp>
#include <glm/exponential.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>

struct Material {
  GLuint diffuse_texture;
  GLuint specular_texture;
  float shininess;

public:
  static Material get_default_material(TextureManager texture_manager) {
    GLuint diffuse;
    texture_manager.load_texture_from_color(gruvbox::orange, diffuse);
    GLuint specular;
    texture_manager.load_texture_from_color(gruvbox::fg, specular);
    // material
    Material material;
    material.diffuse_texture = diffuse;
    material.specular_texture = specular;
    material.shininess = 32;
    return material;
  }
};

struct DirectionalLight {
  glm::vec3 direction;

  glm::vec3 ambient;
  glm::vec3 diffuse;
  glm::vec3 specular;
};

class Background {
public:
  GLuint texture;

  void setup() {
    // gen bind and fill vertex array, vertex buffer object and elemenr buffer
    // object
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(glm::vec2), vertices,
                 GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLuint), indices,
                 GL_STATIC_DRAW);

    // specify how to read vertex buffer object in vertex array
    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2),
                          (void *)0);

    glBindVertexArray(0);
  }

  void draw(Shader &shader) {
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texture);

    shader.set_int("background", 1);

    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
  }

private:
  glm::vec2 vertices[4] = {glm::vec2(0.0, 0.0), glm::vec2(1.0, 0.0),
                           glm::vec2(1.0, 1.0), glm::vec2(0.0, 1.0)};
  GLuint indices[6] = {0, 1, 2, 2, 3, 0};

  GLuint vao, vbo, ebo;
};

class Mesh {
public:
  Mesh();
  Mesh(const std::vector<Vertex> &vertices_, const std::vector<uint> &indices_,
       const Material &material_)
      : vertices(vertices_), indices(indices_), material(material_) {}
  void setup() {
    // gen bind and fill vertex array, vertex buffer object and elemenr buffer
    // object
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
                 &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                 &indices[0], GL_STATIC_DRAW);

    // specify how to read vertex buffer object in vertex array
    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    // normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, normal));
    // uvs
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, uv));
    glBindVertexArray(0);
  }

  void draw(Shader &shader) {
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, material.diffuse_texture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, material.specular_texture);

    shader.set_int("material.diffuse", 1);
    shader.set_int("material.specular", 2);
    shader.set_float("material.shininess", material.shininess);

    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
  }

private:
  std::vector<Vertex> vertices;
  std::vector<GLuint> indices;

  Material material;

  GLuint vao, vbo, ebo;
};

class Transform {
private:
  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 scale_factor{1.0f};
  glm::vec3 offset_position{0.0f};
  glm::quat offset_rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 offset_scale{1.0f};

public:
  glm::mat4 get_transform() const {
    glm::mat4 offset = glm::translate(glm::mat4(1.0f), offset_position) *
                       glm::mat4_cast(offset_rotation) *
                       glm::scale(glm::mat4(1.0f), offset_scale);
    glm::mat4 current = glm::translate(glm::mat4(1.0f), position) *
                        glm::mat4_cast(rotation) *
                        glm::scale(glm::mat4(1.0f), scale_factor);
    return current * offset;
  }

  const glm::vec3 &get_position() const { return position; }

  const glm::vec3 &get_scale() const { return scale_factor; }

  const glm::quat &get_rotation() const { return rotation; }

  const glm::vec3 &get_offset_position() const { return offset_position; }

  const glm::quat &get_offset_rotation() const { return offset_rotation; }

  const glm::vec3 &get_offset_scale() const { return offset_scale; }

  void set_offset_position(const glm::vec3 &new_offset_position) {
    offset_position = new_offset_position;
  }

  void set_offset_rotation(const glm::vec3 &rotation_vector) {
    float angle = glm::length(rotation_vector);
    if (angle == 0.0f) {
      offset_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      return;
    }
    offset_rotation = glm::angleAxis(angle, rotation_vector / angle);
  }

  void set_offset_scale(const glm::vec3 &new_offset_scale) {
    offset_scale = new_offset_scale;
  }

  void move(const glm::vec3 &translation) {
    position += rotation * translation;
  }

  void move_global(const glm::vec3 &translation) { position += translation; }

  void set_position(const glm::vec3 &new_position) { position = new_position; }

  // input: axis-angle vector, angle in radians
  void rotate(const glm::vec3 &rotation_vector) {
    float angle = glm::length(rotation_vector);
    if (angle == 0.0f)
      return;

    glm::vec3 axis = rotation_vector / angle;
    glm::quat delta = glm::angleAxis(angle, axis);

    rotation = glm::normalize(rotation * delta);
  }

  void rotate_global(const glm::vec3 &rotation_vector) {
    float angle = glm::length(rotation_vector);
    if (angle == 0.0f)
      return;

    glm::vec3 axis = rotation_vector / angle;
    glm::quat delta = glm::angleAxis(angle, axis);

    rotation = glm::normalize(delta * rotation);
  }

  void set_rotation(const glm::vec3 &rotation_vector) {
    float angle = glm::length(rotation_vector);

    if (angle == 0.0f) {
      rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      return;
    }

    rotation = glm::angleAxis(angle, rotation_vector / angle);
  }

  void scale(const glm::vec3 &factors) { scale_factor *= factors; }

  void set_scale(const glm::vec3 &new_scale) { scale_factor = new_scale; }
};

class Model {
private:
  std::vector<Mesh> meshes;

  bool load_model(const std::string &model_path,
                  TextureManager &texture_manager) {
    std::filesystem::path obj_path(model_path);
    std::filesystem::path model_dir_path = obj_path.parent_path().parent_path();

    objl::Loader loader;
    bool res = loader.LoadFile(obj_path.string());
    if (!res) {
      fprintf(stderr, "Failed to load model from file %s\n", obj_path.c_str());
      return false;
    }
    for (const auto &l_mesh : loader.LoadedMeshes) {
      std::vector<Vertex> vertices;
      std::vector<GLuint> indices;
      Material material;
      for (const auto &l_vertix : l_mesh.Vertices) {
        Vertex vertex = Vertex();
        vertex.position = glm::vec3(l_vertix.Position.X, l_vertix.Position.Y,
                                    l_vertix.Position.Z);
        vertex.normal =
            glm::vec3(l_vertix.Normal.X, l_vertix.Normal.Y, l_vertix.Normal.Z);
        vertex.uv = glm::vec2(l_vertix.TextureCoordinate.X,
                              l_vertix.TextureCoordinate.Y);
        vertices.push_back(vertex);
      }

      for (const auto &l_indices : l_mesh.Indices) {
        indices.push_back(l_indices);
      }

      if (!l_mesh.MeshMaterial.map_Kd.empty() ||
          !l_mesh.MeshMaterial.map_Ks.empty()) {
        std::filesystem::path diffuse_path(l_mesh.MeshMaterial.map_Kd);
        diffuse_path = model_dir_path / diffuse_path;
        if (std::filesystem::is_regular_file(diffuse_path)) {
          texture_manager.load_texture_from_file(diffuse_path.string(),
                                                 material.diffuse_texture);
        } else {
          fprintf(stderr, "Diffuse texture not found at %s\n",
                  diffuse_path.c_str());
        }
        std::filesystem::path specular_path(l_mesh.MeshMaterial.map_Ks);
        specular_path = model_dir_path / specular_path;
        if (std::filesystem::is_regular_file(specular_path)) {
          texture_manager.load_texture_from_file(specular_path.string(),
                                                 material.specular_texture);
        } else {
          fprintf(stderr, "Specular texture not found at %s\n",
                  specular_path.c_str());
        }
        material.shininess = l_mesh.MeshMaterial.Ns;
      } else {
        material = Material::get_default_material(texture_manager);
      }

      Mesh mesh(vertices, indices, material);
      add_mesh(mesh);
    }
    return true;
  }

public:
  Transform transform;

  Model() {}
  Model(const std::string &model_path, TextureManager &texture_manager) {
    load_model(model_path, texture_manager);
  }
  void add_mesh(Mesh &mesh) {
    mesh.setup();
    meshes.push_back(mesh);
  }

  void draw(Shader &shader) {
    shader.set_matrix_4f("model", transform.get_transform());
    for (Mesh mesh : meshes) {
      mesh.draw(shader);
    }
  }
};

class Renderer {
private:
  std::vector<std::shared_ptr<Model>> models;
  DirectionalLight light;
  Background background = Background();
  bool is_rendering_background = false;
  Shader background_shader =
      Shader(std::string(PROJECT_DIR) + "/src/vert/back.vert",
             std::string(PROJECT_DIR) + "/src/frag/back.frag");
  Shader shader = Shader(std::string(PROJECT_DIR) + "/src/vert/vertex.vert",
                         std::string(PROJECT_DIR) + "/src/frag/fragment.frag");
  std::shared_ptr<Camera> camera;

  void render_models() {
    shader.use();

    shader.set_matrix_4f("view", camera->get_view_matrix());
    shader.set_matrix_4f("projection", camera->get_projection_matrix());
    shader.set_vec3("view_position", camera->get_position());

    shader.set_vec3("light.direction", light.direction);
    shader.set_vec3("light.ambient", light.ambient);
    shader.set_vec3("light.diffuse", light.diffuse);
    shader.set_vec3("light.specular", light.specular);

    for (auto model : models) {
      model->draw(shader);
    }
  }

  void render_background() {
    glDisable(GL_DEPTH_TEST);
    background_shader.use();
    background.draw(background_shader);
    glEnable(GL_DEPTH_TEST);
  }

public:
  Renderer(std::shared_ptr<Camera> camera_) { camera = camera_; }

  bool setup() {

    glClearColor(red_channel(gruvbox::bg), green_channel(gruvbox::bg),
                 blue_channel(gruvbox::bg), 1);
    glEnable(GL_DEPTH_TEST);

    background.setup();

    return shader.build() && background_shader.build();
  }

  void add_model_to_scene(std::shared_ptr<Model> model) {
    models.push_back(model);
  }

  void set_scene_light(DirectionalLight &light_) { light = light_; }

  void set_back_ground_texture(GLuint id) {
    background.texture = id;
    is_rendering_background = true;
  }

  void remove_background_texture() { is_rendering_background = false; }

  void render_scene() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!shader.is_built || !background_shader.is_built) {
      fprintf(
          stderr,
          "Shader programs not built properly, please setup the rederer!\n");
      return;
    }
    if (is_rendering_background) {
      render_background();
    }
    render_models();
  }
};

#endif
