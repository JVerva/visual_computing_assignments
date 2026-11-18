
#include "camera.hpp"
#include "computer_vision.hpp"
#include "geometry.hpp"
#include "input_manager.hpp"
#include "renderer.hpp"
#include "texture_manager.hpp"
#include "utils.hpp"
#include "window.hpp"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <glm/ext/matrix_float4x4.hpp>
#include <opencv2/core.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/detail/qualifier.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/string_cast.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

void show_calibration_scenario(Camera &camera, Renderer &renderer,
                               int scenario_index, std::shared_ptr<Model> cube);

int main() {
  // component initialization
  Window window = Window("Calibration", 640, 480);
  if (!window.init()) {
    fprintf(stderr, "Failed to initialize window!\n");
    return -1;
  }
  InputManager input_manager = InputManager(window);

  TextureManager texture_manager = TextureManager();

  std::shared_ptr<Camera> camera = std::make_shared<Camera>();

  Renderer renderer = Renderer(camera);
  if (!renderer.setup()) {
    fprintf(stderr, "Renderer setup failed!\n");
    return -1;
  }

  std::string calibration_dir =
      std::string(PROJECT_DIR) + "/assets/images/calibration";
  std::vector<cv::Mat> calibration_images;
  load_directory_images(calibration_dir, calibration_images);
  cv::Size pattern_size = cv::Size(10, 7);

  GLuint background_texture;
  create_default_texture(background_texture);

  DirectionalLight light;
  light.direction = glm::normalize(glm::vec3(0.0, 0.0, 1.0));
  light.ambient = glm::vec3(0.2);
  light.diffuse = glm::vec3(0.5);
  light.specular = glm::vec3(1.0);
  renderer.set_scene_light(light);
  // cube creation
  // vertices
  std::vector<Vertex> cube_vertices;
  std::vector<uint> cube_indices;
  get_cube_geometry(1, cube_vertices, cube_indices);
  // material
  Material cube_material = Material::get_default_material(texture_manager);
  // model
  Mesh cube_mesh = Mesh(cube_vertices, cube_indices, cube_material);
  std::shared_ptr<Model> cube = std::make_shared<Model>();
  cube->add_mesh(cube_mesh);
  cube->transform.scale(glm::vec3(1, 1, 0.1));
  cube->transform.move(glm::vec3(0.5, 0.5, -0.05));
  // add cube to scene
  renderer.add_model_to_scene(cube);

  double delta_t;
  double last_frame_t = glfwGetTime();
  while (!window.should_close()) {
    delta_t = last_frame_t - glfwGetTime();
    // camera
    if (input_manager.is_just_pressed(GLFW_KEY_C)) {
      if (!camera->calibrate(calibration_images, pattern_size, true)) {
        fprintf(stderr, "Failed calibration!\n");
      }
      for (int i = 0; i < camera->calibration_info.images.size(); i++) {
        show_calibration_scenario(*camera, renderer, i, cube);
        renderer.render_scene();
        window.swap_buffers();
        last_frame_t = glfwGetTime();
        std::getc(stdin);
      }
    }
    renderer.render_scene();
    input_manager.update();
    window.swap_buffers();
    window.poll_events();
    last_frame_t = glfwGetTime();
  }

  window.close();
}

void show_calibration_scenario(Camera &camera, Renderer &renderer,
                               int scenario_index,
                               std::shared_ptr<Model> cube) {
  if (scenario_index >= camera.calibration_info.images.size()) {
    return;
  }

  GLuint calibration_texture;
  create_default_texture(calibration_texture);
  cv::Mat fliped_img;
  cv::flip(camera.calibration_info.images[scenario_index], fliped_img, 0);

  write_image_to_texture(calibration_texture, fliped_img);
  renderer.set_back_ground_texture(calibration_texture);

  cv::Mat rvec = camera.calibration_info.rotation_vecs[scenario_index];
  cv::Mat tvec = camera.calibration_info.trans_vecs[scenario_index];

  glm::mat4 view = cvt_glm_view(rvec, tvec);
  camera.set_transform_from_view(view);
}
