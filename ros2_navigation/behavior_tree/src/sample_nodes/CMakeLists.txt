add_library(bt_sample_nodes STATIC
    crossdoor_nodes.cpp
    dummy_nodes.cpp
    movebase_node.cpp )

target_link_libraries(bt_sample_nodes ${BEHAVIOR_TREE_LIBRARY})

add_library(crossdoor_nodes_dyn SHARED crossdoor_nodes.cpp )
target_link_libraries(crossdoor_nodes_dyn ${BEHAVIOR_TREE_LIBRARY})

add_library(dummy_nodes_dyn SHARED dummy_nodes.cpp )
target_link_libraries(dummy_nodes_dyn ${BEHAVIOR_TREE_LIBRARY})

add_library(movebase_node_dyn SHARED movebase_node.cpp )
target_link_libraries(movebase_node_dyn ${BEHAVIOR_TREE_LIBRARY})

install(TARGETS 
    bt_sample_nodes
    crossdoor_nodes_dyn
    dummy_nodes_dyn
    movebase_node_dyn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)