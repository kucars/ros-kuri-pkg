FILE(REMOVE_RECURSE
  "CMakeFiles/navigation_gencfg"
  "devel/include/navigation/PIDControllerConfig.h"
  "devel/share/navigation/docs/PIDControllerConfig.dox"
  "devel/share/navigation/docs/PIDControllerConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/navigation/cfg/PIDControllerConfig.py"
  "devel/share/navigation/docs/PIDControllerConfig.wikidoc"
  "devel/include/navigation/MasterControllerConfig.h"
  "devel/share/navigation/docs/MasterControllerConfig.dox"
  "devel/share/navigation/docs/MasterControllerConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/navigation/cfg/MasterControllerConfig.py"
  "devel/share/navigation/docs/MasterControllerConfig.wikidoc"
  "devel/include/navigation/SlaveControllerConfig.h"
  "devel/share/navigation/docs/SlaveControllerConfig.dox"
  "devel/share/navigation/docs/SlaveControllerConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/navigation/cfg/SlaveControllerConfig.py"
  "devel/share/navigation/docs/SlaveControllerConfig.wikidoc"
  "devel/include/navigation/ForceFieldConfig.h"
  "devel/share/navigation/docs/ForceFieldConfig.dox"
  "devel/share/navigation/docs/ForceFieldConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/navigation/cfg/ForceFieldConfig.py"
  "devel/share/navigation/docs/ForceFieldConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/navigation_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
