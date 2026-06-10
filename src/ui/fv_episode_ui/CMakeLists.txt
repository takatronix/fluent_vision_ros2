cmake_minimum_required(VERSION 3.8)
project(fv_episode_ui)

find_package(ament_cmake REQUIRED)

# Install the prebuilt SPA (dist/) as share/fv_episode_ui/web/.
# `npm run build` (run from the host before colcon) populates dist/;
# the dashboard backend exposes this directory at /static/fv_episode_ui/.
install(
  DIRECTORY dist/
  DESTINATION share/${PROJECT_NAME}/web
)

ament_package()
