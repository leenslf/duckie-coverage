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

Checkout [Fields2Cover tutorials](https://fields2cover.github.io/source/tutorials.html) (parts 3-7 are particularly useful)

----
The coverage server accepts `opennav_coverage_msgs/Coordinates[] polygons` by default.