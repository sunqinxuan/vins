find ./vslam/src     -name *.cpp | xargs clang-format -i -style=file
find ./vslam/include -name *.hpp | xargs clang-format -i -style=file
find ./vslam/include -name *.h | xargs clang-format -i -style=file
