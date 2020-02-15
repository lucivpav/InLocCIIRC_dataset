#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
int panoramaWidth = 4098;
int panoramaHeight = 2048;
float angular_resolution_x = 0.08789/1.0, // TODO: use a formula with panorama width
      angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;

Eigen::Vector3f camera_position(9.786105947569013e-04, 1.693258881568909, 0.068662971258163);
Eigen::Vector3f camera_rotation(0.176829305501008, 3.651460591341834, -0.119347990257635);

// --------------
// -----Help-----
// --------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.ply>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-rx <float>  angular resolution in degrees (default "<<angular_resolution_x<<")\n"
            << "-ry <float>  angular resolution in degrees (default "<<angular_resolution_y<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
            << "-h           this help\n"
            << "\n\n";
}

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    // TODO: set up the same default position and orientation
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-l") >= 0)
  {
    live_update = true;
    std::cout << "Live update is on.\n";
  }
  if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
    std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
  if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
    std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  angular_resolution_x = pcl::deg2rad (angular_resolution_x);
  angular_resolution_y = pcl::deg2rad (angular_resolution_y);
    
    Eigen::Vector3f rFix(0.0, 180.0, 180.0);
    camera_rotation += rFix;
    
    for ( int i = 0 ; i < 3 ; i++)
        camera_rotation[i] = pcl::deg2rad(camera_rotation[i]);

    Eigen::Quaternionf camera_orientation =
    Eigen::AngleAxisf(camera_rotation[0], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(camera_rotation[1], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(camera_rotation[2], Eigen::Vector3f::UnitZ());
  
  // ------------------------------------------------------------------
  // -----Read ply file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> ply_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "ply");
  if (!ply_filename_indices.empty ())
  {
    std::string filename = argv[ply_filename_indices[0]];
    if (pcl::io::loadPLYFile (filename, point_cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      printUsage (argv[0]);
      return 0;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (camera_position)) * Eigen::Affine3f (camera_orientation);
  }
  else
  {
    std::cout << "\nNo *.ply file given => Generating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back (point);
      }
    }
    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
  }
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 0;
  pcl::RangeImagePlanar::Ptr range_image_ptr(new pcl::RangeImagePlanar);
  pcl::RangeImagePlanar& range_image = *range_image_ptr;
  //range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
  //                                  pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
  //                                  scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    /* cutout dimensions */
    //int di_width = 1600; 
    //int di_height = 1200;

    /* dimensions in rotatePanoramas */
    int di_width = 1000;
    int di_height = 1000;

    float di_center_x = di_width/2;
    float di_center_y = di_height/2;
    float di_focal_length_x = 500;
    float di_focal_length_y = di_focal_length_x;
    range_image.createFromPointCloudWithFixedSize(point_cloud, di_width, di_height, di_center_x, di_center_y,
                                                  di_focal_length_x, di_focal_length_y, scene_sensor_pose);
    
    // save the range image
    float* ranges = range_image.getRangesArray();
    
//    for (int i = 0 ; i < range_image.width * range_image.height ; i++)
//    {
//        if (ranges[i] != -std::numeric_limits<float>::infinity())
//            std::cout << ranges[i] << std::endl;
//    }

    int nPixels = range_image.width * range_image.height;
    float minDepth = std::numeric_limits<float>::infinity();
    float maxDepth = -std::numeric_limits<float>::infinity();
    for (int i = 0 ; i < nPixels ; i++)
    {
      float depth = ranges[i];
      if (depth == std::numeric_limits<float>::infinity())
        continue;
      if (depth == -std::numeric_limits<float>::infinity())
        continue;
      minDepth = std::min(minDepth, depth);
      maxDepth = std::max(maxDepth, depth);
    }
    std::cout << "minDepth: " << minDepth << ", maxDepth: " << maxDepth << std::endl;
     
    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (ranges, range_image.width, range_image.height,
        minDepth, maxDepth, true);

    std::cout << "range image has dimensions: " << range_image.width << ", " << range_image.height << std::endl;
    pcl::io::saveRgbPNGFile("rangeImage.png", rgb_image, range_image.width, range_image.height);
    std::cout << "saved successfully" << std::endl;
  
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  viewer.addCoordinateSystem (1.0f, "global");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  setViewerPose(viewer, range_image.getTransformationToWorldSystem ());
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer.spinOnce ();
    pcl_sleep (0.01);
    
    if (live_update)
    {
      scene_sensor_pose = viewer.getViewerPose();
      range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
                                        pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                        scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
      range_image_widget.showRangeImage (range_image);
    }
  }
}
