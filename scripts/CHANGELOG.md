# 更新日志

## v2.0.0 - 2025-07-05

### 🆕 新增功能

#### 时间更新功能
- **`--update-dates`** - 仅更新日期相关字段（date 和 lastmod）
- **`--update-lastmod`** - 仅更新最后修改时间
- **`--use-current-time`** - 使用当前时间作为修改时间

#### 智能更新逻辑
- 支持选择性更新特定字段
- 避免不必要的文件修改
- 更精确的跳过逻辑

### 🔧 改进功能

#### 时间处理
- 优化 Git 时间获取逻辑
- 改进时区处理
- 支持当前时间作为修改时间

#### 用户体验
- 更清晰的操作模式提示
- 更详细的预览信息
- 更好的错误处理

### 📝 使用示例

#### 仅更新修改时间
```bash
# 使用当前时间更新所有文件的修改时间
python3 scripts/enhanced_front_matter.py --update-lastmod --use-current-time --force

# 预览将要进行的更改
python3 scripts/enhanced_front_matter.py --update-lastmod --use-current-time --dry-run
```

#### 仅更新日期字段
```bash
# 更新所有文件的创建时间和修改时间
python3 scripts/enhanced_front_matter.py --update-dates --force

# 预览日期更新
python3 scripts/enhanced_front_matter.py --update-dates --dry-run
```

#### 组合使用
```bash
# 强制更新所有文件的修改时间为当前时间
python3 scripts/enhanced_front_matter.py --update-lastmod --use-current-time --force

# 更新单个文件的修改时间
python3 scripts/enhanced_front_matter.py --file "panda-blog/vpn/trojan-vpn.md" --update-lastmod --use-current-time
```

### 🐛 修复问题

- 修复了类型检查错误
- 改进了空值处理
- 优化了时间格式化

### 📚 文档更新

- 更新了 README.md 使用说明
- 添加了 example_usage.md 使用示例
- 新增了 CHANGELOG.md 更新日志

---

## v1.0.0 - 2025-07-05

### 🎉 初始版本

#### 基础功能
- 自动添加创建时间和修改时间
- 自动添加作者信息
- 自动添加分类信息
- 自动生成标题
- 支持 Git 时间获取
- 支持文件系统时间获取

#### 命令行参数
- `--content-dirs` - 指定内容目录
- `--file` - 处理单个文件
- `--force` - 强制更新
- `--dry-run` - 预览模式
- `--author` - 设置作者名称 