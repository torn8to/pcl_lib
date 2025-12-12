#include <thread>

#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <implot.h>


class AsyncIridescenceSLamViewer{
  public:
    AsyncIridescenceSLamViewer(){
        viewer_ = guik::async_viewer();



    }
  private:
    auto viewer_;
}