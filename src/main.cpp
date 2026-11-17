#include "camera.hpp"
#include "computer_vision.hpp"
#include "geometry.hpp"
#include "input_manager.hpp"
#include "renderer.hpp"
#include "texture_manager.hpp"
#include "themes.hpp"
#include "window.hpp"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <glm/detail/qualifier.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/constants.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

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
  camera->set_position(glm::vec3(0, 0, 20));

  Renderer renderer = Renderer(camera);
  if (!renderer.setup()) {
    fprintf(stderr, "Renderer setup failed!\n");
    return -1;
  }

  WebcamCapturePipeline webcam = WebcamCapturePipeline();
  if (!webcam.setup()) {
    fprintf(stderr, "Failed to initialize webcam capture!\n");
    return -1;
  }

  std::string calibration_dir =
      std::string(PROJECT_DIR) + "/assets/images/calibration";
  std::vector<cv::Mat> calibration_images;
  load_directory_images(calibration_dir, calibration_images);
  cv::Size pattern_size = cv::Size(9, 6);

  GLuint background_texture;
  create_default_texture(background_texture);

  DirectionalLight light;
  light.direction = glm::normalize(glm::vec3(0, -1.0, -1.0));
  light.ambient = glm::vec3(0.2);
  light.diffuse = glm::vec3(0.5);
  light.specular = glm::vec3(1.0);
  renderer.set_scene_light(light);

  // model
  std::string model_path =
      std::string(PROJECT_DIR) +
      "/assets/models/steampunk_underwater_explorer/scene.gltf";
  std::shared_ptr<Model> model =
      std::make_shared<Model>(model_path, texture_manager);

  // test transformations
  glm::vec3 translation = glm::vec3(0, 0, 0);
  glm::vec3 scaling = glm::vec3(1);
  glm::vec3 rotation = glm::vec3(0, 0, 0);
  model->move(translation);
  model->rotate(rotation);
  model->scale(scaling);
  // add cube to scene
  renderer.add_model_to_scene(model);

  double delta_t;
  double last_frame_t = glfwGetTime();
  while (!window.should_close()) {
    delta_t = last_frame_t - glfwGetTime();
    webcam.update_frame();
    cv::Mat background_image;
    cv::flip(webcam.frame, background_image, 0);
    write_image_to_texture(background_texture, background_image);
    renderer.set_back_ground_texture(background_texture);

    // input actions, to be removed later when engine architecture is defined
    // webcam
    if (input_manager.is_just_pressed(GLFW_KEY_0))
      webcam.set_filter(Filter::NONE);
    if (input_manager.is_just_pressed(GLFW_KEY_1))
      webcam.set_filter(Filter::GRAYSCALE);
    if (input_manager.is_just_pressed(GLFW_KEY_2))
      webcam.set_filter(Filter::GAUSSIAN_BLUR);
    if (input_manager.is_just_pressed(GLFW_KEY_3))
      webcam.set_filter(Filter::SHARPEN);
    if (input_manager.is_just_pressed(GLFW_KEY_4))
      webcam.set_filter(Filter::SOBEL);
    // camera
    if (input_manager.is_just_pressed(GLFW_KEY_C)) {
      if (!camera->calibrate(calibration_images, pattern_size)) {
        fprintf(stderr, "Failed calibration!\n");
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
