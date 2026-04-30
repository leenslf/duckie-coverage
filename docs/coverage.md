## Setup Instructions

1. **Clone the required repositories:**

   ```bash
   cd  src
   git clone https://github.com/open-navigation/opennav_coverage.git
   git clone https://github.com/Fields2Cover/Fields2Cover.git
   ```

2. **Checkout the correct branches:**

   ```bash
   cd opennav_coverage
   git checkout humble
   cd ../Fields2Cover
   git checkout v1.2.1
   cd ..
   ```
3. **Build and source**
    ```bash
    cd ..
    colcon build
    ```
---

###

* `/coverage_server/coverage_plan` — coverage plan for RViz visualization
* `/coverage_server/field_boundary` — field boundaries for RViz visualization
* `/coverage_server/planning_field` — planning field representation for RViz visualization
* `/coverage_server/swaths` — generated swaths for coverage path for RViz visualization
* `/received_global_plan` — topic published by the controller server

---

Checkout [Fields2Cover tutorials](https://fields2cover.github.io/source/tutorials.html) nice graphics. (parts 3-7 are particularly useful)

----
The coverage server accepts `opennav_coverage_msgs/Coordinates[] polygons` by default.

Take a look at messy code at `src/robot_nav/src/tester.py` to understand coverage client.

The client communicates with `coverage_server` that works with nav2, take a look at messy code at `src/robot_nav/launch/nav2.launch.py`.

Note: "messy code" is messy because I (Leen) copied it as it is from another project, it's not the bible, don't take it as is. 

--- 

In order for this to work we need to have a **server** and a **client**. 
The client gives the server coordinates of the area that needs to be covered (in map frame I guess?). 
The server takes in these coordinates, and plans the path according to the parameters it's given (checkout `coverage_server` at `src/robot_nav/config/nav2_params.yaml` or check [official docs](https://docs.nav2.org/configuration/packages/configuring-coverage-server.html) ).

----
The coverage server also predicts the remaining time / distance remaining to the end. 