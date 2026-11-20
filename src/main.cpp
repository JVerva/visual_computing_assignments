#include "camera.hpp"
#include "input_manager.hpp"
#include "renderer.hpp"
#include "texture_manager.hpp"
#include "window.hpp"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <glm/detail/qualifier.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/constants.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

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
  camera->set_position(glm::vec3(0, 0.25, 1.5));

  Renderer renderer = Renderer(camera);
  if (!renderer.setup()) {
    fprintf(stderr, "Renderer setup failed!\n");
    return -1;
  }

  DirectionalLight light;
  light.direction = glm::normalize(glm::vec3(0, -1.0, -1.0));
  light.ambient = glm::vec3(0.2);
  light.diffuse = glm::vec3(0.5);
  light.specular = glm::vec3(1.0);
  renderer.set_scene_light(light);

  // model
  std::string model_path = std::string(PROJECT_DIR) +
                           "/assets/models/pistol/source/"
                           "pistol.obj";
  std::shared_ptr<Model> model =
      std::make_shared<Model>(model_path, texture_manager);

  // test transformations
  glm::vec3 translation = glm::vec3(0, 0, 0);
  glm::vec3 scaling = glm::vec3(0.1);
  glm::vec3 rotation = glm::vec3(0, glm::quarter_pi<float>(), 0);
  model->transform.move(translation);
  model->transform.set_rotation(rotation);
  model->transform.scale(scaling);
  // add cube to scene
  renderer.add_model_to_scene(model);

  double delta_t;
  double last_frame_t = glfwGetTime();
  while (!window.should_close()) {
    delta_t = last_frame_t - glfwGetTime();
    renderer.render_scene();
    input_manager.update();
    window.swap_buffers();
    window.poll_events();
    last_frame_t = glfwGetTime();
  }

  window.close();
}
