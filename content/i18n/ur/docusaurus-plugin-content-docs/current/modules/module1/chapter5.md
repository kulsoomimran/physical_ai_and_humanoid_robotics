---
sidebar_position: 5
title: "باب 5: ROS 2 پیکجز اور لانچ فائلز"
---

# باب 5: ROS 2 پیکجز اور لانچ فائلز

## جائزہ

یہ باب ROS 2 پیکجز اور لانچ فائلز کے بنیادی تصورات کا جائزہ لیتا ہے، جو ROS 2 ایپلیکیشنز کو منظم، تعمیر اور انجام دینے کے لیے اہم ہیں۔ پیکجز ROS 2 سافٹ ویئر کی منظم کرنے کی بنیادی اکائی فراہم کرتے ہیں، جبکہ لانچ فائلز متعدد نوڈس کے ساتھ پیچیدہ سسٹم کی تشکیل اور انجام دہی کو فعال کرتے ہیں۔

## سیکھنے کے اہداف

اس باب کے اختتام تک، آپ درج ذیل کر سکیں گے:
- مناسب میٹا ڈیٹا کے ساتھ ROS 2 پیکجز تخلیق اور ساخت دینا
- package.xml اور CMakeLists.txt کنفیگریشن فائلز کو سمجھنا
- پیکجز کمپائل کرنے کے لیے colcon بلڈ سسٹم استعمال کرنا
- XML، YAML، اور Python فارمیٹس میں لانچ فائلز تخلیق کرنا
- لانچ فائلز میں نوڈ پیرامیٹرز، نیم سپیسز، اور ری میپنگز تشکیل دینا
- لانچ فائلز کا استعمال کرتے ہوئے پیچیدہ ROS 2 سسٹم منظم کرنا

## میزبان کا مواد

1. [ROS 2 پیکجز کو سمجھنا](#ros-2-پیکجز-کو-سمجھنا)
2. [پیکج ساخت اور تنظیم](#پیکج-ساخت-اور-تنظیم)
3. [پیکج کنفیگریشن فائلز](#پیکج-کنفیگریشن-فائلز)
4. [colcon بلڈ سسٹم](#colcon-بلڈ-سسٹم)
5. [لانچ فائلز تخلیق کرنا](#لانچ-فائلز-تخلیق-کرنا)
6. [لانچ فائل فارمیٹس (XML، YAML، Python)](#لانچ-فائل-فارمیٹس-xml-yaml-python)
7. [لانچ فائلز میں پیرامیٹرز اور نیم سپیسز](#لانچ-فائلز-میں-پیرامیٹرز-اور-نیم-سپیسز)
8. [نوڈ ری میپنگ اور آرگومنٹس](#نوڈ-ری-میپنگ-اور-آرگومنٹس)
9. [لانچ فائل اعلیٰ درجہ کی خصوصیات](#لانچ-فائل-اعلیٰ-درجہ-کی-خصوصیات)
10. [پیکجز اور لانچ فائلز کے لیے بہترین مشقیں](#پیکجز-اور-لانچ-فائلز-کے-لیے-بہترین-مشقیں)
11. [خلاصہ](#خلاصہ)

## ROS 2 پیکجز کو سمجھنا

### منظم کرنے کی بنیادی اکائی کے طور پر پیکج

ROS 2 پیکجز سافٹ ویئر اجزاء کو منظم کرنے کے لیے بنیادی اکائی کے طور پر کام کرتے ہیں۔ ہر پیکج عام طور پر مندرجہ ذیل پر مشتمل ہوتا ہے:
- سورس کوڈ (C++ یا Python)
- کنفیگریشن فائلز
- لانچ فائلز
- دستاویزات
- ٹیسٹس

### پیکج کی اقسام

ROS 2 بلڈ سسٹم کی بنیاد پر مختلف پیکج کی اقسام کی حمایت کرتا ہے:
- **ament_cmake**: CMake استعمال کرنے والے C++ پیکجز کے لیے
- **ament_python**: Python پیکجز کے لیے
- **ament_cargo**: Rust پیکجز کے لیے
- **cmake**: CMake پیکجز کے لیے جو ament استعمال نہیں کرتے

### پیکج انحصار

پیکجز اپنی package.xml فائل میں انحصار کا اعلان کرتے ہیں:
- **buildtool_depend**: بلڈ سسٹم انحصار (مثلاً، ament_cmake)
- **build_depend**: کمپائل کرنے کے لیے ضروری انحصار
- **exec_depend**: رن ٹائم انحصار
- **test_depend**: صرف ٹیسٹ کے لیے ضروری انحصار

## پیکج ساخت اور تنظیم

### CMake پیکج ساخت

CMake پر مبنی ایک معمولی پیکج میں شامل ہوتا ہے:
```
my_package/
├── CMakeLists.txt
├── package.xml
├── include/my_package/
├── src/
│   └── my_node.cpp
└── launch/
    └── my_launch_file.py
```

### Python پیکج ساخت

Python پر مبنی ایک معمولی پیکج میں شامل ہوتا ہے:
```
my_package/
├── package.xml
├── setup.py
├── setup.cfg
├── my_package/
│   ├── __init__.py
│   └── my_node.py
└── launch/
    └── my_launch_file.py
```

### پیکج نامزد کنونشنز

- lowercase کے ساتھ انڈر سکور (snake_case) استعمال کریں
- خالی جگہوں اور خصوصی حروف سے بچیں
- وضاحتی لیکن مختصر رہیں
- ROS نامزد کنونشنز کو فالو کریں

## پیکج کنفیگریشن فائلز

### package.xml

package.xml فائل پیکج کے بارے میں میٹا ڈیٹا مشتمل ہوتا ہے:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My example ROS 2 package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

CMakeLists.txt فائل یہ وضاحت کرتا ہے کہ پیکج کو کیسے تعمیر کرنا ہے:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)

# انحصار تلاش کریں
find_package(rclcpp REQUIRED)

# ایکسیکوٹیبل شامل کریں
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

# ٹارگٹس انسٹال کریں
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## colcon بلڈ سسٹم

### ورک سپیس ساخت

ROS 2 ورک سپیس میں معمولاً یہ ساخت ہوتی ہے:
```
ros2_ws/
├── src/
│   └── my_package/
├── build/
├── install/
└── log/
```

### بنیادی colcon کمانڈز

- `colcon build`: ورک سپیس میں تمام پیکجز تعمیر کریں
- `colcon build --packages-select <package_name>`: مخصوص پیکجز تعمیر کریں
- `colcon build --symlink-install`: تیز تکرار کے لیے سیم لنکس استعمال کریں
- `source install/setup.bash`: تعمیر شدہ پیکجز کو سروش کریں
- `colcon test`: پیکجز کے لیے ٹیسٹ چلائیں

### colcon بلڈ عمل

1. پیکجز کے لیے `src` ڈائریکٹری کو اسکین کریں
2. `build` ڈائریکٹری میں ہر پیکج کو تعمیر کریں
3. آرٹیفیکٹس کو `install` ڈائریکٹری میں انسٹال کریں
4. ماحول کی ترتیب فائلز تیار کریں

## لانچ فائلز تخلیق کرنا

### لانچ فائلز کا مقصد

لانچ فائلز کو فعال کرتا ہے:
- ایک کمانڈ کے ساتھ متعدد نوڈس شروع کرنا
- نوڈ پیرامیٹرز اور آرگومنٹس کی تشکیل
- نیم سپیسز اور ری میپنگز کا قیام
- پیچیدہ سسٹم ڈیپلائمنٹ کا نظم

### لانچ فائل تخلیق عمل

1. `launch/` ڈائریکٹری میں لانچ فائل تخلیق کریں
2. نوڈس اور ان کی کنفیگریشنز کی وضاحت کریں
3. `ros2 launch` کے ساتھ لانچ فائل ٹیسٹ کریں
4. پیکج بلڈ عمل کے ساتھ انضمام

## لانچ فائل فارمیٹس (XML، YAML، Python)

### XML لانچ فائلز

XML لانچ فائلز ایک اعلانیہ ساخت فراہم کرتا ہے:
```xml
<launch>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim"
        namespace="turtlesim1" args="--ros-args --log-level info" />
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose" />
  </node>
</launch>
```

### YAML لانچ فائلز

YAML لانچ فائلز انسانی پڑھنے کے قابل کنفیگریشن پیش کرتا ہے:
```yaml
launch:
  - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "sim"
      namespace: "turtlesim1"
      parameters:
        - {use_sim_time: true}
```

### Python لانچ فائلز

Python لانچ فائلز پروگرامی لچک فراہم کرتا ہے:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
```

## لانچ فائلز میں پیرامیٹرز اور نیم سپیسز

### پیرامیٹرز

لانچ فائلز میں پیرامیٹرز سیٹ کیے جا سکتے ہیں:
- **سٹیٹک پیرامیٹرز**: لانچ ٹائم پر وضاحت شدہ
- **ڈائنامک پیرامیٹرز**: رن ٹائم کے دوران تبدیل کیے جا سکتے ہیں
- **پیرامیٹر فائلز**: پیرامیٹر کنفیگریشنز پر مشتمل YAML فائلز

### نیم سپیسز

نیم سپیسز تنظیم فراہم کرتے ہیں اور نامزد تنازعات سے بچاتے ہیں:
- **گلوبل نیم سپیسز**: لانچ فائل میں تمام نوڈس پر لاگو ہوتا ہے
- **نوڈ مخصوص نیم سپیسز**: انفرادی نوڈس پر لاگو ہوتا ہے
- **ٹوپک ری میپنگ**: ٹوپکس کو نیم سپیس مخصوص راستوں کی طرف ری ڈائریکٹ کرتا ہے

## نوڈ ری میپنگ اور آرگومنٹس

### ٹوپک ری میپنگ

ری میپنگ مختلف ٹوپک نامزد والے نوڈس کو جوڑنے کی اجازت دیتا ہے:
```python
Node(
    package='turtlesim',
    executable='mimic',
    name='mimic',
    remappings=[
        ('/input/pose', '/turtlesim1/turtle1/pose'),
        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    ]
)
```

### کمانڈ لائن آرگومنٹس

لانچ فائلز آرگومنٹس قبول کر سکتے ہیں:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # لانچ آرگومنٹ کا اعلان کریں
    my_param = DeclareLaunchArgument(
        'my_param',
        default_value='default_value',
        description='پیرامیٹر کی تفصیل'
    )

    return LaunchDescription([
        my_param,
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'my_param': LaunchConfiguration('my_param')}]
        )
    ])
```

## لانچ فائل اعلیٰ درجہ کی خصوصیات

### مشروطہ لانچ

لانچ فائلز میں مشروطہ منطق شامل ہو سکتا ہے:
- **اگر شرائط**: پیرامیٹرز کی بنیاد پر نوڈس لانچ کریں
- **مگر شرائط**: پیرامیٹرز کی بنیاد پر نوڈس چھوڑ دیں

### ایونٹ ہینڈلنگ

لانچ فائلز ایونٹس کا جواب دے سکتے ہیں:
- **عملیہ خروج ایونٹس**: نوڈ کریشز یا شٹ ڈاؤن کو ہینڈل کریں
- **ٹائمر ایونٹس**: مخصوص اوقات پر کارروائیوں کو شیڈول کریں
- **کسٹم ایونٹس**: مخصوص رویے کو متحرک کریں

### لانچ شامل کرنا

پیچیدہ سسٹم کو ماڈیولائز کیا جا سکتا ہے:
- **دیگر لانچ فائلز شامل کریں**: لانچ کنفیگریشنز دوبارہ استعمال کریں
- **لانچ فائلز کے درمیان آرگومنٹس پاس کریں**: پیرامیٹرز شیئر کریں
- **نیم سپیس وراثت**: نیسٹڈ نیم سپیسز کا نظم کریں

## پیکجز اور لانچ فائلز کے لیے بہترین مشقیں

### پیکج کی بہترین مشقیں

- مسلسل نامزد کنونشنز کو فالو کریں
- انحصار کو صاف طور پر دستاویز کریں
- مناسب بلڈ اقسام استعمال کریں
- جامع پیکج کی تفصیل شامل کریں
- ٹیسٹس اور دستاویزات شامل کریں

### لانچ فائل کی بہترین مشقیں

- لانچ فائلز کو مرکوز اور ماڈیولر رکھیں
- نوڈس کے لیے وضاحتی نام استعمال کریں
- پیرامیٹر ویلیوز کو دستاویز کریں
- خامیوں کو نرمی سے ہینڈل کریں
- لانچ فائلز کو مکمل طور پر ٹیسٹ کریں

### کارکردگی کے امور

- غیر ضروری انحصار کو کم کریں
- کارآمد بلڈ کنفیگریشنز استعمال کریں
- لانچ فائل انجام کو بہتر بنائیں
- وسائل کا استعمال مانیٹر کریں

## خلاصہ

ROS 2 پیکجز اور لانچ فائلز ROS 2 ایپلیکیشن کی تنظیم اور انجام دہی کی پشت پر ہیں۔ پیکجز کوڈ، انحصار، اور وسائل کو منظم کرنے کی ساخت فراہم کرتے ہیں، جبکہ لانچ فائلز پیچیدہ سسٹم کی تشکیل اور ڈیپلائمنٹ کو فعال کرتے ہیں۔ ان اجزاء کو تخلیق اور استعمال کرنے کا طریقہ سمجھنا قابل برقرار رکھنے اور قابل توسیع ROS 2 ایپلیکیشنز تیار کرنے کے لیے انتہائی اہم ہے۔ colcon بلڈ سسٹم کمپائل کرنے کے عمل کو آسان بناتا ہے، جبکہ مختلف لانچ فائل فارمیٹس (XML، YAML، Python) مختلف استعمال کے معاملات اور ترجیحات کے لیے لچک فراہم کرتے ہیں۔