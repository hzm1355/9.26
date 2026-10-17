# 智能小车 3D Web 可视化

基于 Three.js 的 3D Web 可视化应用，提供类似 Gazebo 的 3D 渲染效果。

## 特性

- ✅ **3D 场景渲染**：使用 Three.js 实现专业级 3D 图形
- ✅ **实时车辆可视化**：5 辆小车在 3D 场景中实时运动
- ✅ **轨道相机控制**：鼠标拖拽旋转、滚轮缩放、右键平移
- ✅ **激光雷达可视化**：实时显示激光雷达扫描数据
- ✅ **车辆信息面板**：显示每辆车的位姿、速度等数据
- ✅ **传感器数据展示**：激光雷达扫描图
- ✅ **响应式 UI**：自适应不同屏幕尺寸

## 快速开始

### 1. 安装依赖

```bash
pip install websockets
```

### 2. 启动服务器

```bash
cd web_3d
python server.py
```

或双击 `start.bat`

### 3. 打开浏览器

访问：http://localhost:8080

## 界面说明

### 3D 场景
- **左键拖拽**：旋转视角
- **滚轮**：缩放
- **右键拖拽**：平移
- **中央车辆**：2x2m 障碍物
- **四周圆柱**：0.5m 半径障碍物

### 左侧面板 - 系统状态
- 仿真状态（运行中/已停止）
- 车辆数量
- 仿真时间
- 实时 FPS

### 右侧面板 - 车辆信息
- 每辆车的实时位姿（X, Y, Yaw）
- 自动更新

### 底部控制栏
- **启动仿真**：开始模拟
- **停止仿真**：暂停模拟
- **重置视角**：恢复默认相机位置
- **俯视图**：切换到顶部视角

### 右下角 - 激光雷达
- 实时显示 smart_car_1 的激光雷达扫描
- 36 条扫描线
- 动态距离变化

## 技术架构

```
浏览器 (Three.js)
    ↓ WebSocket
Python 服务器 (server.py)
    ↓ ROS2 Topics (可选)
Gazebo 仿真
```

### 组件说明

1. **index.html**
   - Three.js 3D 场景
   - 响应式 UI 面板
   - 激光雷达可视化

2. **server.py**
   - HTTP 服务器（提供 HTML/JS）
   - WebSocket 服务器（实时数据）
   - ROS2 桥接（可选）
   - 模拟数据生成器（无 ROS2 时）

## 连接真实仿真

### 前提条件
- WSL2 + ROS2 Humble
- v2x_ws 工作空间已构建
- 仿真正在运行

### 启动步骤

1. **在 WSL2 中启动桥接**
```bash
cd ~/v2x_ws
source install/setup.bash
python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/smart_car_gui/web_3d/server.py
```

2. **在 Windows 浏览器中访问**
```
http://localhost:8080
```

3. **查看真实数据**
   - 车辆位置来自 `/model_states`
   - 可发送 `cmd_vel` 控制车辆

## 自定义场景

### 修改竞技场大小

编辑 `index.html` 中的 `createArenaBoundary()` 函数：
```javascript
const boundaryGeometry = new THREE.BoxGeometry(50, 1, 0.2); // 50m 边长
```

### 添加障碍物

在 `createObstacles()` 函数中添加：
```javascript
const newObstacle = new THREE.Mesh(geometry, material);
newObstacle.position.set(x, y, z);
scene.add(newObstacle);
```

### 修改车辆模型

在 `createVehicleModel()` 函数中调整：
```javascript
const chassisGeometry = new THREE.BoxGeometry(0.8, 0.2, 0.5); // 长宽高
```

### 更换颜色

修改车辆数据：
```javascript
const vehicleData = [
    { name: 'smart_car_1', color: 0x00ff88, pos: [0, -18] }, // 绿色
    // ...
];
```

## 高级功能

### 加载 3D 模型

可以加载 GLTF/GLB 格式的车辆模型：

```javascript
const loader = new THREE.GLTFLoader();
loader.load('models/smart_car.glb', function(gltf) {
    const model = gltf.scene;
    scene.add(model);
});
```

需要添加 GLTFLoader：
```html
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>
```

### 添加纹理

```javascript
const texture = new THREE.TextureLoader().load('textures/ground.jpg');
const groundMaterial = new THREE.MeshStandardMaterial({ 
    map: texture 
});
```

### 粒子效果

添加激光雷达点云：
```javascript
const particles = new THREE.Points();
const geometry = new THREE.BufferGeometry();
// ... 添加点云数据
```

## 性能优化

### 减少渲染负载
- 降低 WebSocket 频率：`await asyncio.sleep(0.1)` (10Hz)
- 简化车辆模型
- 减少阴影质量

### 提升视觉效果
- 启用抗锯齿：`antialias: true`
- 增加阴影分辨率：`shadow.mapSize.width = 4096`
- 添加后处理效果（Bloom, SSAO）

## 故障排除

### 无法访问页面
- 检查端口 8080 是否被占用
- 尝试其他端口：`python server.py 8081`
- 检查防火墙设置

### 3D 场景不显示
- 检查浏览器控制台错误（F12）
- 确认 WebGL 支持
- 尝试其他浏览器（Chrome/Firefox/Edge）

### 数据不更新
- 检查 WebSocket 连接
- 查看浏览器控制台网络标签
- 确认 ROS2 仿真正在运行（如使用真实数据）

## 扩展方向

1. **加载真实 Gazebo 模型**
   - 转换 SDF 到 GLTF
   - 保持视觉一致性

2. **添加传感器可视化**
   - 相机图像
   - IMU 数据
   - 里程计轨迹

3. **交互式控制**
   - 点击车辆选择
   - 键盘控制（WASD）
   - 路径规划可视化

4. **多场景支持**
   - 场景切换菜单
   - 自定义场景编辑器
   - 导入/导出场景

5. **数据分析**
   - 实时图表
   - 历史数据回放
   - 性能统计

## 对比 Gazebo

| 功能 | Gazebo | Web 3D |
|------|--------|--------|
| 3D 渲染 | ★★★★★ | ★★★★☆ |
| 物理仿真 | ★★★★★ | ☆☆☆☆☆ |
| 传感器仿真 | ★★★★★ | ☆☆☆☆☆ |
| 跨平台 | ★★★☆☆ | ★★★★★ |
| 安装难度 | ★★★★☆ | ★☆☆☆☆ |
| 自定义 UI | ★★★☆☆ | ★★★★★ |
| 性能 | ★★★★☆ | ★★★☆☆ |

**总结**：Web 3D 适合可视化和监控，Gazebo 适合物理仿真和传感器测试。

## 许可证

与主项目相同
