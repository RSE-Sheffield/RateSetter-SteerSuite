# Load other subprojects

# Set the projects to appear in the `external` subfolder in IDEs
set(CMAKE_FOLDER "external")


# Add tinyxml as an external project
add_subdirectory(tinyxml)

# Add recastnavigation as an external project
add_subdirectory(recastnavigation)

# If required, add glfw as an external project.
if(ENABLE_GUI AND ENABLE_GLFW)
  add_subdirectory(glfw)
endif()