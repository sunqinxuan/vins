find ./vslam/src     -name *.cpp | xargs clang-format -i -style=file
find ./vslam/src -name *.hpp | xargs clang-format -i -style=file
find ./vslam/src -name *.h | xargs clang-format -i -style=file
