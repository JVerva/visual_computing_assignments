#ifndef RENDERER_HPP
#define RENDERER_HPP

#include "camera.hpp"
#include "cgltf.h"
#include "geometry.hpp"
#include "shader.hpp"
#include "texture_manager.hpp"
#include "themes.hpp"
#include <cstddef>
#include <cstdio>
#include <glad/glad.h>
#include <glm/detail/qualifier.hpp>
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

class Model {
private:
  std::vector<Mesh> meshes;
  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 scale_factor{1.0f};

  bool load_model(std::string model_path, TextureManager texture_manager) {
    cgltf_result res;
    cgltf_options opt = {cgltf_file_type_invalid};
    cgltf_data *data = nullptr;
    res = cgltf_parse_file(&opt, model_path.c_str(), &data);
    if (res != cgltf_result_success) {
      fprintf(stderr, "Failed to parse glTF file %s!\n", model_path.c_str());
      return false;
    }
    res = cgltf_load_buffers(&opt, data, model_path.c_str());
    if (res != cgltf_result_success) {
      fprintf(stderr, "Failed to load glTF file %s!\n", model_path.c_str());
      cgltf_free(data);
      return false;
    }
    res = cgltf_validate(data);
    if (res != cgltf_result_success) {
      fprintf(stderr, "Failed to validate glTF data from file %s!\n",
              model_path.c_str());
      cgltf_free(data);
      return false;
    }

    for (cgltf_size i = 0; i < data->meshes_count; i++) {
      const auto &gltf_mesh = data->meshes[i];
      for (cgltf_size j = 0; j < gltf_mesh.primitives_count; j++) {
        const auto &primitive = gltf_mesh.primitives[j];
        if (primitive.type != cgltf_primitive_type_triangles) {
          fprintf(stderr, "Skipping unsupported primitive mode in mesh %zu.\n",
                  static_cast<size_t>(i));
          continue;
        }

        const cgltf_accessor *positions = nullptr;
        const cgltf_accessor *normals = nullptr;
        const cgltf_accessor *uvs = nullptr;

        for (cgltf_size k = 0; k < primitive.attributes_count; k++) {
          const cgltf_attribute &attribute = primitive.attributes[k];
          switch (attribute.type) {
          case cgltf_attribute_type_position:
            positions = attribute.data;
            break;
          case cgltf_attribute_type_normal:
            normals = attribute.data;
            break;
          case cgltf_attribute_type_texcoord:
            if (attribute.index == 0)
              uvs = attribute.data;
            break;
          default:
            break;
          }
        }

        if (positions == nullptr || positions->type != cgltf_type_vec3) {
          fprintf(stderr, "Skipping mesh %zu primitive without positions.\n",
                  static_cast<size_t>(i));
          continue;
        }

        if ((normals != nullptr && normals->count != positions->count) ||
            (uvs != nullptr && uvs->count != positions->count)) {
          fprintf(stderr,
                  "Skipping mesh %zu primitive with mismatched attributes.\n",
                  static_cast<size_t>(i));
          continue;
        }

        std::vector<Vertex> vertices(positions->count);
        bool attributes_read = true;
        for (cgltf_size k = 0; k < positions->count; k++) {
          cgltf_float position[3];
          cgltf_float normal[3] = {0.0f, 1.0f, 0.0f};
          cgltf_float uv[2] = {0.0f, 0.0f};

          attributes_read &=
              cgltf_accessor_read_float(positions, k, position, 3) != 0;
          if (normals != nullptr)
            attributes_read &=
                cgltf_accessor_read_float(normals, k, normal, 3) != 0;
          if (uvs != nullptr)
            attributes_read &= cgltf_accessor_read_float(uvs, k, uv, 2) != 0;

          vertices[k] = {{position[0], position[1], position[2]},
                         {normal[0], normal[1], normal[2]},
                         {uv[0], uv[1]}};
        }

        if (!attributes_read) {
          fprintf(stderr, "Failed to read attributes for mesh %zu.\n",
                  static_cast<size_t>(i));
          continue;
        }

        std::vector<GLuint> indices;
        if (primitive.indices != nullptr) {
          indices.resize(primitive.indices->count);
          for (cgltf_size k = 0; k < primitive.indices->count; k++)
            indices[k] = static_cast<GLuint>(
                cgltf_accessor_read_index(primitive.indices, k));
        } else {
          indices.resize(vertices.size());
          for (size_t k = 0; k < indices.size(); k++)
            indices[k] = static_cast<GLuint>(k);
        }

        GLuint diffuse;
        texture_manager.load_texture_from_color(gruvbox::orange, diffuse);
        GLuint specular;
        texture_manager.load_texture_from_color(gruvbox::fg, specular);
        // material
        Material material;
        material.diffuse_texture = diffuse;
        material.specular_texture = specular;
        material.shininess = 32;
        Mesh mesh(vertices, indices, material);
        add_mesh(mesh);
      }
    }

    cgltf_free(data);
    return true;
  }

public:
  Model() {}
  Model(std::string model_path, TextureManager texture_manager) {
    load_model(model_path, texture_manager);
  }
  void add_mesh(Mesh &mesh) {
    mesh.setup();
    meshes.push_back(mesh);
  }

  // Build the transformation matrix: T * R * S
  glm::mat4 get_transform() const {
    return glm::translate(glm::mat4(1.0f), position) *
           glm::mat4_cast(rotation) * glm::scale(glm::mat4(1.0f), scale_factor);
  }

  const glm::vec3 &get_position() const { return position; }

  const glm::vec3 &get_scale() const { return scale_factor; }

  const glm::quat &get_rotation() const { return rotation; }

  void move(const glm::vec3 &translation) {
    position += rotation * translation;
  }

  void move_global(const glm::vec3 &translation) { position += translation; }

  void set_position(const glm::vec3 &new_position) { position = new_position; }

  void set_transform(const glm::mat4 &transform) {
    position = glm::vec3(transform[3]);
    rotation = glm::quat_cast(transform);
    scale_factor = glm::vec3(1.0f);
  }

  // Input: axis-angle vector, angle in radians
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

  void draw(Shader &shader) {
    shader.set_matrix_4f("model", get_transform());
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
