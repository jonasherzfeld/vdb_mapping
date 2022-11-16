// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \author  Lennart Puck puck@fzi.de
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------

// RACK KION - Robotics Application Construction Kit (KION internal)
// Copyright (C) 2022 KION Group AG
//
// All rights reserved.
//
// Author
//     Jonas Herzfeld <jonas.herzfeld@kiongroup.com>

#ifndef VDB_MAPPING_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_VDB_MAPPING_H_INCLUDED

#include <Eigen/Geometry>

#include <openvdb/Types.h>
#include <openvdb/io/Stream.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Morphology.h>

#include <perception/scan3d_proxy.h>

namespace vdb_mapping {


/*!
 * \brief Accumulation of configuration parameters
 */
struct BaseConfig
{
  double max_range;
  std::string map_directory_path;
};
/*!
 * \brief Main Mapping class which handles all data integration
 */
template <typename DataT, typename ConfigT = BaseConfig>
class VDBMapping
{
public:
  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;

  using GridT       = openvdb::Grid<typename openvdb::tree::Tree4<DataT, 5, 4, 3>::Type>;
  using UpdateGridT = openvdb::Grid<openvdb::tree::Tree4<bool, 1, 4, 3>::Type>;


  VDBMapping()                  = delete;
  VDBMapping(const VDBMapping&) = delete;
  VDBMapping& operator=(const VDBMapping&) = delete;

  /*!
   * \brief Construktur creates a new VDBMapping objekt with parametrizable grid resolution
   *
   * \param resolution Resolution of the VDB Grid
   */
  VDBMapping(const double resolution);
  virtual ~VDBMapping(){};

  /*!
   * \brief Creates a new VDB Grid
   *
   * \param resolution Resolution of the grid
   *
   * \returns Grid shared pointer
   */
  typename GridT::Ptr createVDBMap(double resolution);

  /*!
   * \brief Reset the current map
   */
  void resetMap();

  /*!
   * \brief Saves the current map
   */
  bool saveMap() const;

  /*!
   * \brief Loads a stored map
   */
  bool loadMap(const std::string file_path);

  /*!
   * \brief Handles the integration of new Scan3d data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   *
   * \returns Was the insertion of the new Scan3d successful
   */
  bool insertPointCloud(const scan3d_data* cloud,
                        const Eigen::Matrix<double, 3, 1>& origin);

  /*!
   * \brief Handles the integration of new Scan3d data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   * \param update_grid Update grid that was created internally while maping
   *
   * \returns Was the insertion of the new scan successful
   */
  bool insertPointCloud(const scan3d_data* cloud,
                        const Eigen::Matrix<double, 3, 1>& origin,
                        UpdateGridT::Ptr& update_grid,
                        UpdateGridT::Ptr& raycast_update_grid,
                        UpdateGridT::Ptr& overwrite_grid,
                        const bool reduce_data);

  /*!
   * \brief Creates a grid which contains all cells which should be updated by the
   * inserted scan.
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   *
   * \returns Bitmask Grid containing all cells which have to be updated
   */
  UpdateGridT::Ptr createUpdate(const scan3d_data* cloud,
                                const Eigen::Matrix<double, 3, 1>& origin) const;

  /*!
   * \brief  Raycasts a Scan3d into an update Grid
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the sensor measurement
   *
   * \returns Raycasted update grid
   */
  void raycastPointCloud(const scan3d_data* cloud,
                         const Eigen::Matrix<double, 3, 1>& origin,
                         UpdateGridT::Ptr& temp_grid) const;
  /*!
   * \brief Raycasts an reduced data update Grid into full update grid
   *
   * \param grid Reduced data update Grid containing only the endpoints of the input sensor data
   *
   * \returns Full update Grid
   */
  void raycastUpdateGrid(const UpdateGridT::Ptr& grid,
                         UpdateGridT::Ptr& temp_grid) const;

  /*!
   * \brief Creates a reduced data update grid from a scan which only contains the
   * endpoints of the input cloud
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the senosr measurement
   *
   * \returns Reduced update grid
   */
  void pointCloudToUpdateGrid(const scan3d_data* cloud,
                              const Eigen::Matrix<double, 3, 1>& origin,
                              UpdateGridT::Ptr& temp_grid) const;
  /*!
   * \brief Casts a single ray into an update grid structure
   *
   * \param ray_origin_world Ray origin in world coordinates
   * \param ray_origin_index Ray origin in index coordinates
   * \param ray_end_world Ray endpoint in world coordinates
   * \param update_grid_acc Accessor to the update grid
   */
  void castRayIntoGrid(const openvdb::Vec3d& ray_origin_world,
                       const Vec3T& ray_origin_index,
                       openvdb::Vec3d& ray_end_world,
                       UpdateGridT::Accessor& update_grid_acc) const;

  /*!
   * \brief Overwrites the active states of a map given an update grid
   *
   * \param update_grid Update Grid containing all states that changed during the last update
   */
  void overwriteMap(const UpdateGridT::Ptr& update_grid);

  /*!
   * \brief Incorporates the information of an update grid to the internal map. This will update the
   * probabilities of all cells specified by the update grid.
   *
   * \param temp_grid Grid containing all cells which shall be updated
   *
   * \returns Was the insertion of the scan successful
   */
  void updateMap(const UpdateGridT::Ptr& temp_grid,
                 UpdateGridT::Ptr& change);
  void updateMap(const UpdateGridT::Ptr& temp_grid);

  /*!
   * \brief Returns a pointer to the VDB map structure
   *
   * \returns Map pointer
   */
  typename GridT::Ptr getMap() const { return m_vdb_grid; }

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  virtual void setConfig(const ConfigT& config);


protected:
  virtual bool updateFreeNode(DataT& voxel_value, bool& active) { return false; }
  virtual bool updateOccupiedNode(DataT& voxel_value, bool& active) { return false; }
  /*!
   * \brief VDB grid pointer
   */
  typename GridT::Ptr m_vdb_grid;
  /*!
   * \brief Maximum raycasting distance
   */
  double m_max_range;
  /*!
   * \brief Grid resolution of the map
   */
  double m_resolution;
  /*!
   * \brief path where the maps will be stored
   */
  std::string m_map_directory_path;
  /*!
   * \brief Flag checking wether a valid config was already loaded
   */
  bool m_config_set;
};

#include "VDBMapping.hpp"

} // namespace vdb_mapping

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
