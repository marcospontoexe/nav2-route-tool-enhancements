// Copyright (c) 2025 Open Navigation LLC
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
//
// Modified in 2026 by Marcos Daniel Santana Grochoska and Erico Meger (Route Tool enhancements, see
// README.md in the repository root).

#include <limits>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <any>
#include <typeinfo>
#include <algorithm>
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_route/types.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/line_iterator.hpp"

#ifndef NAV2_ROUTE__UTILS_HPP_
#define NAV2_ROUTE__UTILS_HPP_

namespace nav2_route
{

namespace utils
{

/**
 * @brief Convert the position into a pose
 * @param x X Coordinates
 * @param y Y Coordinates
 * @return PoseStamped of the position
 */
inline geometry_msgs::msg::PoseStamped toMsg(const float x, const float y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

/**
 * @brief Convert the route graph into a visualization marker array for visualization
 * @param graph Graph of nodes and edges
 * @param frame Frame ID to use
 * @param now Current time to use
 * @return MarkerArray of the graph
 */
inline visualization_msgs::msg::MarkerArray toMsg(
  const nav2_route::Graph & graph, const std::string & frame, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker nodes_marker;
  nodes_marker.header.frame_id = frame;
  nodes_marker.header.stamp = now;
  nodes_marker.action = 0;
  nodes_marker.ns = "route_graph_nodes";
  nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker.scale.x = 0.1;
  nodes_marker.scale.y = 0.1;
  nodes_marker.scale.z = 0.1;
  nodes_marker.color.r = 1.0;
  nodes_marker.color.a = 1.0;
  nodes_marker.points.reserve(graph.size());

  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = frame;
  edges_marker.header.stamp = now;
  edges_marker.action = 0;
  edges_marker.ns = "route_graph_edges";
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.scale.x = 0.05;  // Line width
  edges_marker.color.g = 1.0;
  edges_marker.color.a = 0.5;  // Semi-transparent green so bidirectional connections stand out
  constexpr size_t points_per_edge = 2;
  // This probably under-reserves but saves some initial reallocations
  constexpr size_t likely_min_edges_per_node = 2;
  edges_marker.points.reserve(graph.size() * points_per_edge * likely_min_edges_per_node);

  geometry_msgs::msg::Point node_pos;
  geometry_msgs::msg::Point edge_start;
  geometry_msgs::msg::Point edge_end;

  visualization_msgs::msg::Marker node_id_marker;
  node_id_marker.header.frame_id = frame;
  node_id_marker.header.stamp = now;
  node_id_marker.action = 0;
  node_id_marker.ns = "route_graph_node_ids";
  node_id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  node_id_marker.scale.x = 0.1;
  node_id_marker.scale.y = 0.15;
  node_id_marker.scale.z = 0.15;
  node_id_marker.color.a = 1.0;
  node_id_marker.color.r = 1.0;  // Yellow (red + green): stands out more than upstream's red
  node_id_marker.color.g = 1.0;
  node_id_marker.color.b = 0.0;
  node_id_marker.pose.position.z = 0.3;  // Raise node IDs above the node spheres

  visualization_msgs::msg::Marker edge_id_marker;
  edge_id_marker.header.frame_id = frame;
  edge_id_marker.header.stamp = now;
  edge_id_marker.action = 0;
  edge_id_marker.ns = "route_graph_edge_ids";
  edge_id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  edge_id_marker.scale.x = 0.12;  // Slightly larger than upstream's 0.1
  edge_id_marker.scale.y = 0.12;
  edge_id_marker.scale.z = 0.12;
  edge_id_marker.color.a = 1.0;
  edge_id_marker.color.r = 0.0;  // Blue: unidirectional default (pairs recolored below)
  edge_id_marker.color.g = 0.2;
  edge_id_marker.color.b = 1.0;
  edge_id_marker.pose.position.z = 0.2;  // Raise edge IDs above the edge lines

  visualization_msgs::msg::Marker node_id_bg_marker;
  node_id_bg_marker.header.frame_id = frame;
  node_id_bg_marker.header.stamp = now;
  node_id_bg_marker.action = 0;
  node_id_bg_marker.ns = "route_graph_node_ids_bg";
  node_id_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
  node_id_bg_marker.scale.x = 0.18;  // Slightly larger than the text
  node_id_bg_marker.scale.y = 0.08;
  node_id_bg_marker.scale.z = 0.001;  // Thin plate
  node_id_bg_marker.color.a = 0.8;  // Semi-transparent
  node_id_bg_marker.color.r = 0.0;  // Black
  node_id_bg_marker.color.g = 0.0;
  node_id_bg_marker.color.b = 0.0;
  node_id_bg_marker.pose.position.z = 0.3;  // Just behind the text

  visualization_msgs::msg::Marker edge_id_bg_marker;
  edge_id_bg_marker.header.frame_id = frame;
  edge_id_bg_marker.header.stamp = now;
  edge_id_bg_marker.action = 0;
  edge_id_bg_marker.ns = "route_graph_edge_ids_bg";
  edge_id_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
  edge_id_bg_marker.scale.x = 0.15;
  edge_id_bg_marker.scale.y = 0.08;
  edge_id_bg_marker.scale.z = 0.001;
  edge_id_bg_marker.color.a = 0.8;
  edge_id_bg_marker.color.r = 0.0;
  edge_id_bg_marker.color.g = 0.0;
  edge_id_bg_marker.color.b = 0.0;
  edge_id_bg_marker.pose.position.z = 0.19;

  visualization_msgs::msg::Marker edge_arrow_marker;
  edge_arrow_marker.header.frame_id = frame;
  edge_arrow_marker.header.stamp = now;
  edge_arrow_marker.action = 0;
  edge_arrow_marker.ns = "route_graph_edge_arrows";
  edge_arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  edge_arrow_marker.scale.x = 0.15;  // Arrow length
  edge_arrow_marker.scale.y = 0.03;  // Arrow width
  edge_arrow_marker.scale.z = 0.03;  // Arrow height
  edge_arrow_marker.color.a = 0.8;  // Overwritten per edge: alpha comes from the "penalty"
  edge_arrow_marker.color.r = 1.0;  // Orange by default; recolored per edge below
  edge_arrow_marker.color.g = 0.5;
  edge_arrow_marker.color.b = 0.0;
  edge_arrow_marker.pose.position.z = 0.05;  // Slightly above the ground

  // Background plate of the speed_limit label (colored with the edge color)
  visualization_msgs::msg::Marker edge_speed_bg_marker;
  edge_speed_bg_marker.header.frame_id = frame;
  edge_speed_bg_marker.header.stamp = now;
  edge_speed_bg_marker.action = 0;
  edge_speed_bg_marker.ns = "route_graph_edge_speed_limits_bg";
  edge_speed_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
  edge_speed_bg_marker.scale.x = 0.30;  // Wide enough for "100.0%"
  edge_speed_bg_marker.scale.y = 0.10;
  edge_speed_bg_marker.scale.z = 0.001;  // Thin plate
  edge_speed_bg_marker.color.a = 0.85;  // r/g/b set per edge from edge_color
  edge_speed_bg_marker.pose.position.z = 0.24;

  // speed_limit value text, drawn near the start node of the edge
  visualization_msgs::msg::Marker edge_speed_marker;
  edge_speed_marker.header.frame_id = frame;
  edge_speed_marker.header.stamp = now;
  edge_speed_marker.action = 0;
  edge_speed_marker.ns = "route_graph_edge_speed_limits";
  edge_speed_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  edge_speed_marker.scale.x = 0.12;
  edge_speed_marker.scale.y = 0.12;
  edge_speed_marker.scale.z = 0.12;
  edge_speed_marker.color.r = 0.0;  // Fixed black: contrasts with all three
  edge_speed_marker.color.g = 0.0;  // possible backgrounds (orange,
  edge_speed_marker.color.b = 0.0;  // green/yellow, blue), all fairly light
  edge_speed_marker.color.a = 1.0;
  edge_speed_marker.pose.position.z = 0.25;

  // ---- Arrow transparency from the edge "penalty" (PenaltyScorer) ---- The LOWER the penalty, the
  // more transparent the arrow; the HIGHER, the more opaque. Only the arrows change: they never
  // overlap (the two arrows of a bidirectional pair start at the same midpoint and point in
  // opposite directions) and each arrow is its own marker, with its own material in RViz, so one
  // arrow's alpha never affects another's (unlike the edges' LINE_LIST, whose material is shared
  // and whose last point decides the transparency of all lines).
  constexpr float arrow_alpha_min = 0.5f;  // penalty 0: high floor so the direction stays readable
  constexpr float arrow_alpha_max = 1.0f;  // highest penalty in the graph: fully opaque

  // Read the penalty of an edge. Missing -> 0.0 (same default as PenaltyScorer). Only floats are
  // accepted: "penalty": 50 written without a decimal point is parsed as an int, and PenaltyScorer
  // throws bad_any_cast on that edge (the route fails); here we only warn once and treat it as 0.0,
  // so the visualization keeps working.
  auto edge_penalty = [](const nav2_route::DirectionalEdge & edge) -> float {
      auto it = edge.metadata.data.find("penalty");
      if (it == edge.metadata.data.end()) {
        return 0.0f;
      }
      if (it->second.type() != typeid(float)) {
        RCLCPP_WARN_ONCE(
          rclcpp::get_logger("nav2_route_utils"),
          "Edge metadata key 'penalty' is not stored as a float (found type: %s); "
          "treating it as 0.0 for the arrow transparency. Graph files should write "
          "penalty as a decimal (e.g. 50.0, not 50) -- the PenaltyScorer also fails "
          "on such edges.",
          it->second.type().name());
        return 0.0f;
      }
      return std::any_cast<float>(it->second);
    };

  // First pass: highest penalty in the graph, to normalize the transparency (0..1) relative to the
  // graph itself, so the signature of toMsg() (used by route_server and by the Route Tool panel)
  // does not need to change.
  float max_penalty = 0.0f;
  for (const auto & node : graph) {
    for (const auto & edge : node.neighbors) {
      max_penalty = std::max(max_penalty, edge_penalty(edge));
    }
  }

  for (const auto & node : graph) {
    node_pos.x = node.coords.x;
    node_pos.y = node.coords.y;
    nodes_marker.points.push_back(node_pos);

    // Add background for Node ID
    node_id_bg_marker.id++;
    node_id_bg_marker.pose.position.x = node.coords.x + 0.15;
    node_id_bg_marker.pose.position.y = node.coords.y + 0.15;
    msg.markers.push_back(node_id_bg_marker);

    // Add text for Node ID
    node_id_marker.id++;
    node_id_marker.pose.position.x = node.coords.x + 0.15;
    node_id_marker.pose.position.y = node.coords.y + 0.15;
    node_id_marker.text = std::to_string(node.nodeid);
    msg.markers.push_back(node_id_marker);

    for (const auto & neighbor : node.neighbors) {
      edge_start.x = node.coords.x;
      edge_start.y = node.coords.y;
      edge_end.x = neighbor.end->coords.x;
      edge_end.y = neighbor.end->coords.y;

      // Check whether the reverse edge exists (bidirectional pair)
      bool is_bidirectional = false;
      for (const auto & reverse_neighbor : neighbor.end->neighbors) {
        if (reverse_neighbor.end->nodeid == node.nodeid) {
          is_bidirectional = true;
          break;
        }
      }

      // Edge color by type (shared by the arrow and the speed_limit label background)
      std_msgs::msg::ColorRGBA edge_color;
      edge_color.a = 1.0;
      if (is_bidirectional && node.nodeid > neighbor.end->nodeid) {
        edge_color.r = 0.3; edge_color.g = 0.7; edge_color.b = 1.0;  // return edge: blue
      } else if (is_bidirectional) {
        edge_color.r = 0.5; edge_color.g = 1.0; edge_color.b = 0.3;  // outbound edge: green/yellow
      } else {
        edge_color.r = 1.0; edge_color.g = 0.5; edge_color.b = 0.0;  // unidirectional edge: orange
      }

      edges_marker.points.push_back(edge_start);
      edges_marker.points.push_back(edge_end);

      // Deal with overlapping bi-directional text markers by offsetting locations
      float y_offset = 0.0;
      if (node.nodeid > neighbor.end->nodeid) {
        y_offset = 0.08;  // Wider than upstream's 0.05 to fit the label backgrounds
      } else {
        y_offset = -0.08;
      }
      const float x_offset = 0.12;  // Wider than upstream's 0.07 to fit the label backgrounds

      // Bidirectional pairs get their own ID colors
      visualization_msgs::msg::Marker current_edge_id_bg = edge_id_bg_marker;
      visualization_msgs::msg::Marker current_edge_id = edge_id_marker;

      if (is_bidirectional && node.nodeid > neighbor.end->nodeid) {
        // "Return" edge of a bidirectional pair: light blue text on dark blue
        current_edge_id_bg.color.r = 0.0;
        current_edge_id_bg.color.g = 0.3;
        current_edge_id_bg.color.b = 0.6;
        current_edge_id_bg.color.a = 0.9;

        current_edge_id.color.r = 0.3;
        current_edge_id.color.g = 0.7;
        current_edge_id.color.b = 1.0;
      } else if (is_bidirectional) {
        // "Outbound" edge of a bidirectional pair: green/yellow text on dark green
        current_edge_id_bg.color.r = 0.2;
        current_edge_id_bg.color.g = 0.5;
        current_edge_id_bg.color.b = 0.0;
        current_edge_id_bg.color.a = 0.9;

        current_edge_id.color.r = 0.5;
        current_edge_id.color.g = 1.0;
        current_edge_id.color.b = 0.3;
      }
      // else: unidirectional edges keep the default blue

      // Add background for Edge ID
      current_edge_id_bg.id = edge_id_bg_marker.id + 1;
      edge_id_bg_marker.id++;
      current_edge_id_bg.pose.position.x =
        node.coords.x + ((neighbor.end->coords.x - node.coords.x) / 2.0) + x_offset;
      current_edge_id_bg.pose.position.y =
        node.coords.y + ((neighbor.end->coords.y - node.coords.y) / 2.0) + y_offset;
      msg.markers.push_back(current_edge_id_bg);

      // Add text for Edge ID
      current_edge_id.id = edge_id_marker.id + 1;
      edge_id_marker.id++;
      current_edge_id.pose.position.x =
        node.coords.x + ((neighbor.end->coords.x - node.coords.x) / 2.0) + x_offset;
      current_edge_id.pose.position.y =
        node.coords.y + ((neighbor.end->coords.y - node.coords.y) / 2.0) + y_offset;
      current_edge_id.text = std::to_string(neighbor.edgeid);
      msg.markers.push_back(current_edge_id);

      // Add background + text for the edge's speed_limit near the start node, only if the edge
      // actually has that metadata
      auto speed_it = neighbor.metadata.data.find("speed_limit");
      if (speed_it != neighbor.metadata.data.end()) {
        if (speed_it->second.type() == typeid(float)) {
          const float speed_limit_value = std::any_cast<float>(speed_it->second);

          const float edge_dx = neighbor.end->coords.x - node.coords.x;
          const float edge_dy = neighbor.end->coords.y - node.coords.y;
          const float edge_len = std::hypotf(edge_dx, edge_dy);

          float ux = 0.0, uy = 0.0;
          if (edge_len > 1e-3) {
            ux = edge_dx / edge_len;
            uy = edge_dy / edge_len;
          }
          const float perp_x = -uy;  // perpendicular to the edge direction
          const float perp_y = ux;

          // Walk a small fraction of the edge from the start node (not the midpoint, where the ID
          // is), with a floor and a cap so the label neither sticks to the node on short edges nor
          // slides to the midpoint on long ones
          float speed_along = std::max(0.25f * edge_len, 0.25f);
          speed_along = std::min(speed_along, 0.4f * edge_len);

          // Same side convention as the ID's y_offset, to keep the label off the edge line
          const float speed_side = (node.nodeid > neighbor.end->nodeid) ? 1.0f : -1.0f;

          const float speed_x = node.coords.x + ux * speed_along +
            perp_x * 0.12f * speed_side;
          const float speed_y = node.coords.y + uy * speed_along +
            perp_y * 0.12f * speed_side;

          visualization_msgs::msg::Marker current_edge_speed_bg = edge_speed_bg_marker;
          current_edge_speed_bg.id = ++edge_speed_bg_marker.id;
          current_edge_speed_bg.pose.position.x = speed_x;
          current_edge_speed_bg.pose.position.y = speed_y;
          current_edge_speed_bg.color.r = edge_color.r;
          current_edge_speed_bg.color.g = edge_color.g;
          current_edge_speed_bg.color.b = edge_color.b;
          msg.markers.push_back(current_edge_speed_bg);

          // speed_limit is a percentage of the maximum speed (see AdjustSpeedLimit), hence the "%"
          // suffix
          std::ostringstream speed_text_stream;
          speed_text_stream << std::fixed << std::setprecision(1) << speed_limit_value << "%";

          visualization_msgs::msg::Marker current_edge_speed_text = edge_speed_marker;
          current_edge_speed_text.id = ++edge_speed_marker.id;
          current_edge_speed_text.pose.position.x = speed_x;
          current_edge_speed_text.pose.position.y = speed_y;
          current_edge_speed_text.text = speed_text_stream.str();
          msg.markers.push_back(current_edge_speed_text);
        } else {
          RCLCPP_WARN_ONCE(
            rclcpp::get_logger("nav2_route_utils"),
            "Edge metadata key 'speed_limit' is not stored as a float (found type: %s); "
            "skipping the speed limit label for one or more edges. Graph files should "
            "write speed_limit as a decimal (e.g. 85.0, not 85) so it is parsed as a float.",
            speed_it->second.type().name());
        }
      }

      // Add arrow for Edge ID
      edge_arrow_marker.id++;

      // The arrow sits at the midpoint of the edge
      float arrow_x = node.coords.x + ((neighbor.end->coords.x - node.coords.x) / 2.0);
      float arrow_y = node.coords.y + ((neighbor.end->coords.y - node.coords.y) / 2.0);

      edge_arrow_marker.pose.position.x = arrow_x;
      edge_arrow_marker.pose.position.y = arrow_y;

      // Point the arrow along the edge direction (start -> end)
      float dx = neighbor.end->coords.x - node.coords.x;
      float dy = neighbor.end->coords.y - node.coords.y;
      float yaw = std::atan2(dy, dx);

      // Yaw-only quaternion
      edge_arrow_marker.pose.orientation.x = 0.0;
      edge_arrow_marker.pose.orientation.y = 0.0;
      edge_arrow_marker.pose.orientation.z = std::sin(yaw / 2.0);
      edge_arrow_marker.pose.orientation.w = std::cos(yaw / 2.0);

      // Color the arrow by edge type (the hue still tells outbound / return / unidirectional)
      edge_arrow_marker.color = edge_color;

      // Transparency from the penalty of THIS edge (this direction), normalized by the highest
      // penalty in the graph: t = 0 (no penalty) ... 1 (highest penalty). A graph without any
      // penalty leaves every arrow at the floor (arrow_alpha_min).
      const float penalty_t = (max_penalty > 0.0f) ?
        std::clamp(edge_penalty(neighbor) / max_penalty, 0.0f, 1.0f) : 0.0f;
      edge_arrow_marker.color.a =
        arrow_alpha_min + (arrow_alpha_max - arrow_alpha_min) * penalty_t;

      msg.markers.push_back(edge_arrow_marker);
    }
  }

  msg.markers.push_back(edges_marker);
  msg.markers.push_back(nodes_marker);
  return msg;
}

/**
 * @brief Convert the route into a message
 * @param route Route of nodes and edges
 * @param frame Frame ID to use
 * @param now Current time to use
 * @return Route message
 */
inline nav2_msgs::msg::Route toMsg(
  const nav2_route::Route & route, const std::string & frame, const rclcpp::Time & now)
{
  nav2_msgs::msg::Route msg;
  msg.header.frame_id = frame;
  msg.header.stamp = now;
  msg.route_cost = route.route_cost;

  nav2_msgs::msg::RouteNode route_node;
  nav2_msgs::msg::RouteEdge route_edge;
  route_node.nodeid = route.start_node->nodeid;
  route_node.position.x = route.start_node->coords.x;
  route_node.position.y = route.start_node->coords.y;
  msg.nodes.push_back(route_node);

  // Provide the Node info and Edge IDs we're traversing through
  for (unsigned int i = 0; i != route.edges.size(); i++) {
    route_edge.edgeid = route.edges[i]->edgeid;
    route_edge.start.x = route.edges[i]->start->coords.x;
    route_edge.start.y = route.edges[i]->start->coords.y;
    route_edge.end.x = route.edges[i]->end->coords.x;
    route_edge.end.y = route.edges[i]->end->coords.y;
    msg.edges.push_back(route_edge);

    route_node.nodeid = route.edges[i]->end->nodeid;
    route_node.position.x = route.edges[i]->end->coords.x;
    route_node.position.y = route.edges[i]->end->coords.y;
    msg.nodes.push_back(route_node);
  }

  return msg;
}

/**
 * @brief Finds the normalized dot product of 2 vectors
 * @param v1x Vector 1's x component
 * @param v1y Vector 1's y component
 * @param v2x Vector 2's x component
 * @param v2y Vector 2's y component
 * @return Value of dot product
 */
inline float normalizedDot(
  const float v1x, const float v1y,
  const float v2x, const float v2y)
{
  const float mag1 = std::hypotf(v1x, v1y);
  const float mag2 = std::hypotf(v2x, v2y);
  if (mag1 < 1e-6 || mag2 < 1e-6) {
    return 0.0;
  }
  return (v1x / mag1) * (v2x / mag2) + (v1y / mag1) * (v2y / mag2);
}

/**
 * @brief Finds the closest point on the line segment made up of start-end to pose
 * @param pose Pose to find point closest on the line with respect to
 * @param start Start of line segment
 * @param end End of line segment
 * @return Coordinates of point on the line closest to the pose
 */
inline Coordinates findClosestPoint(
  const geometry_msgs::msg::PoseStamped & pose,
  const Coordinates & start, const Coordinates & end)
{
  Coordinates pt;
  const float vx = end.x - start.x;
  const float vy = end.y - start.y;
  const float ux = start.x - pose.pose.position.x;
  const float uy = start.y - pose.pose.position.y;
  const float uv = vx * ux + vy * uy;
  const float vv = vx * vx + vy * vy;

  // They are the same point, so only one option
  if (vv < 1e-6) {
    return start;
  }

  const float t = -uv / vv;
  if (t > 0.0 && t < 1.0) {
    pt.x = (1.0 - t) * start.x + t * end.x;
    pt.y = (1.0 - t) * start.y + t * end.y;
  } else if (t <= 0.0) {
    pt = start;
  } else {
    pt = end;
  }

  return pt;
}

inline float distance(const Coordinates & coords, const geometry_msgs::msg::PoseStamped & pose)
{
  return hypotf(coords.x - pose.pose.position.x, coords.y - pose.pose.position.y);
}

}  // namespace utils

}  // namespace nav2_route

#endif  // NAV2_ROUTE__UTILS_HPP_
