set(target CnoidROSPlugin)

cnoid_make_gettext_mofiles(${target} mofiles)

add_cnoid_plugin(${target} SHARED
  ROSPlugin.cpp
  BodyPublisherItem.cpp
  ${mofiles}
  )

target_link_libraries(${target}
  #${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
  )

install(TARGETS ${target}
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}
  )

if(CHOREONOID_ENABLE_PYTHON)
  if(CHOREONOID_USE_PYBIND11)
    add_subdirectory(pybind11)
  endif()
endif()
