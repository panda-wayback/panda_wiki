# 使用示例

## 快速开始

### 1. 预览模式（推荐首次使用）

```bash
# 预览所有文件的更改，不实际修 --dry-run
```

输出示例：
```
📁 找到 7 个markdown文件
👤 作者: panda
📂 内容目录: panda-blog
🔍 预览模式 - 显示将要进行的更改:
  panda-blog/vpn/trojan-vpn.md:
    创建时间: 2025-07-05 02:09:51
    作者: panda
    分类: 技术文档, VPN技术
    标题: Trojan Vpn
  ...
```

### 2. 实际执行

```bash
# 为所有文件添加 Front Matter
python3 scripts/enhanced_front_matter.py
```

输出示例：
```
📁 找到 7 个markdown文件
👤 作者: panda
📂 内容目录: panda-blog
✅ 更新 trojan-vpn.md
   📅 创建时间: 2025-07-05 02:09:51
   🔄 修改时间: 2025-07-05 02:09:51
   👤 作者: panda
   📂 分类: 技术文档, VPN技术
   📝 标题: Trojan Vpn
...
📊 更新完成: 7/7 个文件
```

### 3. 处理单个文件

```bash
# 处理特定文件
python3 scripts/enhanced_front_matter.py --file "panda-blog/vpn/trojan-vpn.md"
```

### 4. 自定义作者

```bash
# 设置自定义作者名称
python3 scripts/enhanced_front_matter.py --author "你的名字"
```

### 5. 强制更新

```bash
# 强制更新所有文件（包括已有 Front Matter 的文件）
python3 scripts/enhanced_front_matter.py --force
```

### 6. 时间更新功能

```bash
# 仅更新日期相关字段（date 和 lastmod）
python3 scripts/enhanced_front_matter.py --update-dates

# 仅更新最后修改时间
python3 scripts/enhanced_front_matter.py --update-lastmod

# 使用当前时间作为修改时间
python3 scripts/enhanced_front_matter.py --update-lastmod --use-current-time

# 强制更新所有文件的修改时间为当前时间
python3 scripts/enhanced_front_matter.py --update-lastmod --use-current-time --force
```

## 生成的 Front Matter 示例

### VPN 技术文章
```yaml
---
date: "2025-07-05T02:09:51+0800"
author: panda
categories: ["技术文档", "VPN技术"]
title: "Trojan Vpn"
lastmod: "2025-07-05T02:09:51+0800"
---
```

### 工具使用文章
```yaml
---
date: "2025-07-01T12:14:44+0800"
author: panda
categories: ["技术文档", "工具使用"]
title: "Proxmox"
lastmod: "2025-07-01T12:14:44+0800"
---
```

### 创意想法文章
```yaml
---
date: "2025-07-01T12:14:44+0800"
author: panda
categories: ["技术文档", "创意想法"]
title: "履带车农村自动配送"
lastmod: "2025-07-01T12:14:44+0800"
---
```

## 分类映射

脚本会根据文件路径自动生成分类：

| 目录路径 | 生成的分类 |
|---------|-----------|
| `panda-blog/` | 技术文档 |
| `panda-blog/vpn/` | 技术文档, VPN技术 |
| `panda-blog/工具/` | 技术文档, 工具使用 |
| `panda-blog/想法/` | 技术文档, 创意想法 |
| `panda-blog/ACM/` | 技术文档, 算法竞赛 |
| `panda-blog/macos/` | 技术文档, macOS |
| `panda-blog/server/` | 技术文档, 服务器 |

## 注意事项

1. **首次使用建议先预览**：使用 `--dry-run` 参数查看将要进行的更改
2. **备份重要文件**：虽然脚本会保留原有内容，但建议备份重要文件
3. **Git 仓库**：在 Git 仓库中运行可以获得更准确的时间信息
4. **编码支持**：脚本支持 UTF-8 编码的中文文件名和内容 