#include "camera.hpp"
#include "computer_vision.hpp"
#include "feature_tracker.hpp"
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
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/matrix.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#define TRACKER_FEATURES 1000

const std::string TARGET_IMAGE_PATH =
    std::string(PROJECT_DIR) + "/assets/images/feature_matching/id_1.png";
const std::string MODEL_PATH = std::string(PROJECT_DIR) +
                               "/assets/models/pistol/source/"
                               "pistol.obj";

enum Mode { FEATURE, POSE };

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
  const glm::vec3 default_camera_position = glm::vec3(0, 0, -10);
  camera->set_position(default_camera_position);

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

  // calibrate camera
  std::string calibration_dir =
      std::string(PROJECT_DIR) + "/assets/images/calibration";
  std::vector<cv::Mat> calibration_images;
  load_directory_images(calibration_dir, calibration_images);
  cv::Size pattern_size = cv::Size(10, 7);
  if (!camera->calibrate(calibration_images, pattern_size)) {
    fprintf(stderr, "Failed calibration!\n");
  }

  cv::Mat target_image = cv::imread(TARGET_IMAGE_PATH);
  FeatureTracker sift_feature_tracker =
      SIFTFeatureTracker(TRACKER_FEATURES, target_image);
  sift_feature_tracker.set_camera_calibration(
      camera->calibration_info.intrinsic, camera->calibration_info.dist_coefs);
  FeatureTracker orb_feature_tracker =
      ORBFeatureTracker(TRACKER_FEATURES, target_image);
  orb_feature_tracker.set_camera_calibration(
      camera->calibration_info.intrinsic, camera->calibration_info.dist_coefs);
  FeatureTracker surf_feature_tracker = SURFFeatureTracker(target_image);
  surf_feature_tracker.set_camera_calibration(
      camera->calibration_info.intrinsic, camera->calibration_info.dist_coefs);
  FeatureTracker akaze_feature_tracker = AKAZEFeatureTracker(target_image);
  akaze_feature_tracker.set_camera_calibration(
      camera->calibration_info.intrinsic, camera->calibration_info.dist_coefs);
  GLuint background_texture;
  create_default_texture(background_texture);

  Mode mode = FEATURE;
  FeatureTracker feature_tracker = sift_feature_tracker;

  // light creation
  DirectionalLight light;
  light.direction = glm::normalize(glm::vec3(0.0, 0.0, 1.0));
  light.ambient = glm::vec3(0.2);
  light.diffuse = glm::vec3(0.5);
  light.specular = glm::vec3(1.0);
  renderer.set_scene_light(light);

  // model
  std::shared_ptr<Model> model =
      std::make_shared<Model>(MODEL_PATH, texture_manager);
  const glm::vec3 default_model_position = glm::vec3(0.5, 0.5, 0);
  const glm::vec3 default_model_rotation =
      glm::vec3(0, -glm::half_pi<float>(), 0);
  model->transform.set_offset_position(default_model_position);
  model->transform.set_offset_rotation(default_model_rotation);
  model->transform.set_offset_scale(glm::vec3(0.3, 0.3, 0.3));
  // add cube to scene
  renderer.add_model_to_scene(model);

  double last_frame_t = glfwGetTime();
  while (!window.should_close()) {
    double frame_start = glfwGetTime();
    double frame_delta = frame_start - last_frame_t;
    last_frame_t = frame_start;

    double capture_start = glfwGetTime();
    webcam.update_frame();
    double capture_end = glfwGetTime();

    double tracking_start = glfwGetTime();
    feature_tracker.set_input_image(webcam.frame);
    feature_tracker.visualy_track(true);
    double tracking_end = glfwGetTime();
    // get input
    if (input_manager.is_just_pressed(GLFW_KEY_N)) {
      mode = POSE;
    }
    if (input_manager.is_just_pressed(GLFW_KEY_M)) {
      mode = FEATURE;
    }
    if (input_manager.is_just_pressed(GLFW_KEY_0)) {
      feature_tracker = sift_feature_tracker;
    }
    if (input_manager.is_just_pressed(GLFW_KEY_1)) {
      feature_tracker = orb_feature_tracker;
    }
    if (input_manager.is_just_pressed(GLFW_KEY_2)) {
      feature_tracker = surf_feature_tracker;
    }
    if (input_manager.is_just_pressed(GLFW_KEY_3)) {
      feature_tracker = akaze_feature_tracker;
    }
    // change background
    cv::Mat background_image;
    switch (mode) {
    case FEATURE:
      cv::flip(feature_tracker.get_match_image(), background_image, 0);
      write_image_to_texture(background_texture, background_image);
      camera->set_position(default_camera_position);
      break;
    case POSE:
      cv::flip(webcam.frame, background_image, 0);
      write_image_to_texture(background_texture, background_image);
      auto pose = feature_tracker.get_estimated_pose();
      auto rvec = get<0>(pose);
      auto tvec = get<1>(pose);
      if (rvec.empty() or tvec.empty()) {
        break;
      }

      glm::mat4 view = cvt_glm_view(rvec, tvec);
      camera->set_transform_from_view(view);
      break;
    }
    renderer.set_back_ground_texture(background_texture);

    double render_start = glfwGetTime();
    renderer.render_scene();
    double render_end = glfwGetTime();
    input_manager.update();

    double swap_start = glfwGetTime();
    window.swap_buffers();
    double swap_end = glfwGetTime();
    window.poll_events();

    fprintf(stdout,
            "frame: %.2f ms, capture: %.2f ms, tracking: %.2f ms, "
            "render: %.2f ms, swap: %.2f ms\n",
            frame_delta * 1000.0, (capture_end - capture_start) * 1000.0,
            (tracking_end - tracking_start) * 1000.0,
            (render_end - render_start) * 1000.0,
            (swap_end - swap_start) * 1000.0);
  }

  window.close();
}
