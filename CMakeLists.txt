cmake_minimum_required(VERSION 3.14)
project(udp_communicator)

# 必須パッケージの読み込み
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# インクルードディレクトリの追加（ヘッダ用）
include_directories(
  include
)

# 実行ファイルの定義（main + クラス実装）
add_executable(udp_communicator
  src/main.cpp
  src/udp_communicator.cpp
)

# リンクするライブラリや依存パッケージ
ament_target_dependencies(udp_communicator
  rclcpp
  std_msgs
  nlohmann_json
)

# インストール設定（ビルド後にROSパスに置く）
install(TARGETS udp_communicator
  DESTINATION lib/${PROJECT_NAME}
)

# include/ ディレクトリのインストール（他パッケージで使えるように）
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
