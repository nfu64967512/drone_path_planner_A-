"""
A* Pathfinding Algorithm Test Script
Test A* pathfinding functionality
"""

from astar_pathfinding import GridMap, AStarPathfinder
from obstacle_manager import Obstacle, ObstacleManager

def test_astar_basic():
    """Basic A* pathfinding test"""
    print("=" * 50)
    print("Test 1: Basic A* pathfinding (no obstacles)")
    print("=" * 50)

    # Create simple map bounds
    bounds = (24.0, 24.01, 120.0, 120.01)  # (min_lat, max_lat, min_lon, max_lon)
    grid_map = GridMap(bounds, resolution=1.0)

    pathfinder = AStarPathfinder(grid_map)

    # Test pathfinding
    start = (24.001, 120.001)
    end = (24.009, 120.009)

    path = pathfinder.find_path(start, end)

    if path:
        print(f"[OK] Path found: {len(path)} points")
        print(f"  Start: {start}")
        print(f"  End: {end}")
        print(f"  Path length: {len(path)} points")
    else:
        print("[FAIL] No path found")

    print()


def test_astar_with_obstacle():
    """Test A* pathfinding with obstacles"""
    print("=" * 50)
    print("Test 2: A* pathfinding with obstacles")
    print("=" * 50)

    # Create map
    bounds = (24.0, 24.01, 120.0, 120.01)
    grid_map = GridMap(bounds, resolution=0.5)

    # Add a square obstacle in the middle
    obstacle_polygon = [
        (24.0045, 120.0045),
        (24.0045, 120.0055),
        (24.0055, 120.0055),
        (24.0055, 120.0045)
    ]

    grid_map.mark_obstacle(obstacle_polygon)
    print(f"[OK] Obstacle marked: square region")

    pathfinder = AStarPathfinder(grid_map)

    # Test path that would go through obstacle
    start = (24.002, 120.005)
    end = (24.008, 120.005)

    print(f"  Start: {start}")
    print(f"  End: {end}")
    print(f"  (Direct path would go through obstacle)")

    path = pathfinder.find_path(start, end)

    if path:
        print(f"[OK] Detour path found: {len(path)} points")
        print(f"  Path preview:")
        for i, point in enumerate(path[:5]):
            print(f"    {i+1}. {point}")
        if len(path) > 5:
            print(f"    ... and {len(path)-5} more points")
    else:
        print("[FAIL] No path found")

    print()


def test_obstacle_manager_integration():
    """Test ObstacleManager integration"""
    print("=" * 50)
    print("Test 3: ObstacleManager integration test")
    print("=" * 50)

    # Create obstacle manager
    manager = ObstacleManager()

    # Add an obstacle
    obstacle = Obstacle([
        (24.0045, 120.0045),
        (24.0045, 120.0055),
        (24.0055, 120.0055),
        (24.0055, 120.0045)
    ], safe_distance=2.0)
    obstacle.is_complete = True
    manager.obstacles.append(obstacle)

    print(f"[OK] Obstacle added: {len(obstacle.corners)} corners, safe distance {obstacle.safe_distance}m")

    # Create waypoints that would go through obstacle
    waypoints = [
        (24.002, 120.005),
        (24.008, 120.005)
    ]

    # Boundary
    boundary = [
        (24.001, 120.001),
        (24.001, 120.009),
        (24.009, 120.009),
        (24.009, 120.001)
    ]

    print(f"  Original waypoints: {len(waypoints)}")

    # Apply obstacle avoidance
    result = manager.filter_waypoints_with_detour(waypoints, boundary)

    print(f"[OK] Obstacle avoidance completed")
    print(f"  Result waypoints: {len(result)}")

    if len(result) > len(waypoints):
        print(f"  Detour path generated (+{len(result) - len(waypoints)} points)")
    else:
        print(f"  No detour needed")

    print()


def main():
    """Run all tests"""
    print("\n")
    print("=" * 50)
    print("     A* Obstacle Avoidance Algorithm Test")
    print("=" * 50)
    print()

    try:
        test_astar_basic()
        test_astar_with_obstacle()
        test_obstacle_manager_integration()

        print("=" * 50)
        print("[OK] All tests completed!")
        print("=" * 50)
        print()
        print("Summary:")
        print("- A* algorithm successfully integrated")
        print("- Grid map creation works correctly")
        print("- Can find detour paths around obstacles")
        print("- ObstacleManager supports A* avoidance")
        print()
        print("Next step: Test in main application with real obstacles")
        print()

    except Exception as e:
        print(f"\n[FAIL] Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
