#include <spirit/spGeneralTools.h>

// Checks if a file exists in a path
bool spGeneralTools::CheckFileExists(const std::string& file_name) {
  struct stat buffer;
  return(stat(file_name.c_str(),&buffer) == 0);
}

void spGeneralTools::Delay_ms(unsigned int delay_time) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
}

void spGeneralTools::Delay_us(unsigned int delay_time) {
  std::this_thread::sleep_for(std::chrono::microseconds(delay_time));
}

spTimestamp spGeneralTools::Tick(){
  return std::chrono::high_resolution_clock::now();
}

double spGeneralTools::Tock_ms(spTimestamp tick_time) {
  spTimestamp tock_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::Tock_us(spTimestamp tick_time) {
  spTimestamp tock_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::TickTock_ms(spTimestamp tick_time,spTimestamp tock_time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::TickTock_us(spTimestamp tick_time,spTimestamp tock_time) {
  return std::chrono::duration_cast<std::chrono::microseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::TickTock_s(spTimestamp tick_time,spTimestamp tock_time) {
  return std::chrono::duration_cast<std::chrono::seconds>( tock_time - tick_time ).count();
}

void spGeneralTools::PlotXY(std::vector<double>& x_axis, std::string xlabel, std::string ylabel) {
  pangolin::CreateWindowAndBind("PlotXY Window",640,480);

  // Data logger object
  pangolin::DataLog log;

  // Optionally add named labels
  std::vector<std::string> labels;
  labels.push_back(xlabel);
  labels.push_back(ylabel);
  log.SetLabels(labels);
  const double tinc = 0.01;
  double min_value = 0;
  double max_value = 0;
  for(int ii=0; ii<x_axis.size(); ii++) {
    log.Log(x_axis[ii]);
    if(x_axis[ii]<min_value)
      min_value = x_axis[ii];
    if(x_axis[ii]>max_value)
      max_value = x_axis[ii];
  }
  double x_diff = max_value-min_value;
  x_diff *= 0.1;
  // OpenGL 'view' of data. We might have many views of the same data.
  pangolin::Plotter plotter(&log,-1,1,min_value-x_diff,max_value+x_diff,0.01,1);
  plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
//  plotter.Track("$i");
//  pangolin::XYRangef window_range();

//  plotter.SetDefaultView(window_range);
  pangolin::DisplayBase().AddDisplay(plotter);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while(!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Render graph, Swap frames and Process Events
    pangolin::FinishFrame();
  }

}
