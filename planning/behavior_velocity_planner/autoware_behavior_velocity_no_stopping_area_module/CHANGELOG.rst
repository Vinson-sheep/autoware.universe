^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_no_stopping_area_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(planning): move package directory for planning factor interface (`#9948 <https://github.com/autowarefoundation/autoware.universe/issues/9948>`_)
  * chore: add new package for planning factor interface
  * chore(surround_obstacle_checker): update include file
  * chore(obstacle_stop_planner): update include file
  * chore(obstacle_cruise_planner): update include file
  * chore(motion_velocity_planner): update include file
  * chore(bpp): update include file
  * chore(bvp-common): update include file
  * chore(blind_spot): update include file
  * chore(crosswalk): update include file
  * chore(detection_area): update include file
  * chore(intersection): update include file
  * chore(no_drivable_area): update include file
  * chore(no_stopping_area): update include file
  * chore(occlusion_spot): update include file
  * chore(run_out): update include file
  * chore(speed_bump): update include file
  * chore(stop_line): update include file
  * chore(template_module): update include file
  * chore(traffic_light): update include file
  * chore(vtl): update include file
  * chore(walkway): update include file
  * chore(motion_utils): remove factor interface
  ---------
* feat(behavior_velocity_planner)!: remove velocity_factor completely (`#9943 <https://github.com/autowarefoundation/autoware.universe/issues/9943>`_)
  * feat(behavior_velocity_planner)!: remove velocity_factor completely
  * minimize diff
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware.universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* feat(behavior_velocity_modules): add node test (`#9790 <https://github.com/autowarefoundation/autoware.universe/issues/9790>`_)
  * feat(behavior_velocity_crosswalk): add node test
  * fix
  * feat(behavior_velocity_xxx_module): add node test
  * fix
  * fix
  * fix
  * fix
  * change directory tests -> test
  ---------
* refactor(behavior_velocity_planner_common): add behavior_velocity_rtc_interface and move RTC-related implementation (`#9799 <https://github.com/autowarefoundation/autoware.universe/issues/9799>`_)
  * split into planer_common and rtc_interface
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_rtc_interface/include/autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * fix
  ---------
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
* Contributors: Fumiya Watanabe, Mamoru Sobue, Satoshi OTA, Takayuki Murooka

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware.universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware.universe/issues/9588>`_)
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware.universe/issues/9570>`_)
* feat(behavior_velocity_planner)!: remove stop_reason (`#9452 <https://github.com/autowarefoundation/autoware.universe/issues/9452>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* feat(behavior_velocity_planner): replace first_stop_path_point_index (`#9296 <https://github.com/autowarefoundation/autoware.universe/issues/9296>`_)
  * feat(behavior_velocity_planner): replace first_stop_path_point_index
  * add const
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/src/scene.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner/src/planner_manager.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_behavior_velocity_no_stopping_area_module): fix cppcheck knownConditionTrueFalse (`#9189 <https://github.com/autowarefoundation/autoware.universe/issues/9189>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Mamoru Sobue, Ryohsuke Mitsudome, Ryuta Kambe, Yukinari Hisaki, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware.universe (`#9304 <https://github.com/autowarefoundation/autoware.universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware.universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware.universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_behavior_velocity_no_stopping_area_module): fix cppcheck knownConditionTrueFalse (`#9189 <https://github.com/autowarefoundation/autoware.universe/issues/9189>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* test(no_stopping_area): refactor and add tests (`#9009 <https://github.com/autowarefoundation/autoware.universe/issues/9009>`_)
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware.universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/autowarefoundation/autoware.universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* fix(autoware_behavior_velocity_no_stopping_area_module): fix uninitMemberVar (`#8321 <https://github.com/autowarefoundation/autoware.universe/issues/8321>`_)
  fix:uninitMemberVar
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware.universe/issues/7640>`_)
* fix(autoware_behavior_velocity_planner_common): remove lane_id check from arc_lane_util (`#7710 <https://github.com/autowarefoundation/autoware.universe/issues/7710>`_)
  * fix(arc_lane_util): remove lane_id check from arc_lane_util
  * modify test_arc_lane_util.cpp
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware.universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware.universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware.universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/autowarefoundation/autoware.universe/issues/7526>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, Maxime CLEMENT, Takayuki Murooka, Yukinari Hisaki, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
