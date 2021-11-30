---
title: OpenSSF专题之四：criticality_score
date: 2021-11-29 11:06:07
tags:
---

# 简介

- Open Source Project Criticality Score(Beta)

  该项目目前由Securing Critical Projects WG组织来维护https://github.com/ossf/wg-securing-critical-projects

  该组织成立的动机：大大小小的开源项目都面临着资源分配的问题，包括所需的时间、资源和注意力。需要一个将关键项目与可为其提供支持的组织联系起来的方法。

- 目标
  - 为每一个开源项目生成一个关键性分数（临界分数）
  - 创建开源社区所依赖的关键项目列表
  - 使用这些数据主动改善这些关键项目的安全状况

# 核心算法

关键性分数（临界分数）：定义了项目的影响力和重要性，是一个介于**0（最不重要）**和**1（最重要）**之间的数字。

来源于Rob Pike的量化关键性（Quantifying Criticality）算法。https://github.com/ossf/criticality_score/blob/main/Quantifying_criticality_algorithm.pdf

![img](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题之四：criticality-score/formula.png)

我们使用以下参数来推导出开源项目的关键性分数：

| 参数 (S i )           | 重量 (α i ) | 最大阈值 (T i ) | 描述                                   | 推理                                                         |
| --------------------- | ----------- | --------------- | -------------------------------------- | ------------------------------------------------------------ |
| created_since         | 1           | 120             | 项目创建的时间（以月为单位）           | 较旧的项目有更高的机会被广泛使用或依赖。                     |
| updated_since         | -1          | 120             | 自项目上次更新以来的时间（以月为单位） | 最近没有提交的未维护项目有更高的机会不那么依赖。             |
| **contributor_count** | **2**       | 5000            | 项目贡献者的数量（有提交）             | 不同的贡献者参与表明了项目的重要性。                         |
| org_count             | 1           | 10              | 贡献者所属的不同组织的数量             | 表示跨组织依赖性。                                           |
| commit_frequency      | 1           | 1000            | 去年平均每周提交次数                   | 较高的代码流失稍微表明了项目的重要性。此外，对漏洞的敏感性更高。 |
| recent_releases_count | 0.5         | 26              | 去年发布的数量                         | 频繁发布表明用户依赖。重量较轻，因为这并不总是使用。         |
| closed_issues_count   | 0.5         | 5000            | 过去 90 天内关闭的问题数量             | 表示贡献者高度参与并专注于解决用户问题。较低的权重，因为它依赖于项目贡献者。 |
| updated_issues_count  | 0.5         | 5000            | 过去 90 天内更新的问题数量             | 表示贡献者参与度高。较低的权重，因为它依赖于项目贡献者。     |
| comment_frequency     | 1           | 15              | 过去 90 天内每个问题的平均评论数       | 表示用户活跃度和依赖性高。                                   |
| **dependents_count**  | **2**       | 500000          | 在commit 消息中提及的项目数量          | 表示存储库使用，通常在版本卷中。此参数适用于所有语言，包括没有包依赖关系图的 C/C++（虽然是 hack-ish）。计划在不久的将来添加包依赖树。 |

# 用法

- 安装criticality-score模块

  ```
  pip3 install criticality-score
  ```

- 运行该模块，该模块只需要一个参数-仓库名称

  ```
  $ criticality_score --repo github.com/kubernetes/kubernetes
  
  name: kubernetes
  url: https://github.com/kubernetes/kubernetes
  language: Go
  description: Production-Grade Container Scheduling and Management
  created_since: 87
  updated_since: 0
  contributor_count: 3999
  watchers_count: 79583
  org_count: 5
  commit_frequency: 97.2
  recent_releases_count: 70
  updated_issues_count: 5395
  closed_issues_count: 3062
  comment_frequency: 5.5
  dependents_count: 454393
  criticality_score: 0.99107
  ```

**该模块已经在仓库中实现，使用python开发，目前可以支持github与gitlab。**

仓库中使用import github以及import gitlab去获取仓库内所需要的信息，主要是提取url，当然需要创建并授权github访问token与gitlab访问token，然后分别保存到环境变量GITHUB_AUTH_TOKEN、GITHUB_AUTH_TOKEN中。创建方法参考https://docs.github.com/en/developers/apps/getting-started-with-apps/about-apps#personal-access-tokens与https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html

```
# For posix platforms, e.g. linux, mac:
export GITHUB_AUTH_TOKEN=<your access token>

# For windows:
set GITHUB_AUTH_TOKEN=<your access token>
```

```
# For posix platforms, e.g. linux, mac:
export GITLAB_AUTH_TOKEN=<your access token>

# For windows:
set GITLAB_AUTH_TOKEN=<your access token>
```

支持的格式：default, json, csv

# 公共数据

- 如果只对关键项目的临界分数列表感兴趣，官方就会以csv格式发布它们。数据存储在Google Cloud Storage上，可以通过gsutil命令行工具下载，或者从浏览器[此处](https://commondatastorage.googleapis.com/ossf-criticality-score/index.html)下载

  ```
  $ gsutil ls gs://ossf-criticality-score/ * .csv
  gs://ossf-criticality-score/c_top_200.csv
  gs://ossf-criticality-score/cplusplus_top_200.csv
  gs://ossf-criticality-score/csharp_top_200.csv
  gs://ossf-criticality-score/go_top_200.csv
  gs://ossf-criticality-score/java_top_200.csv
  gs://ossf-criticality-score/js_top_200.csv
  gs://ossf-criticality-score/php_top_200.csv
  gs://ossf-criticality-score/python_top_200.csv
  gs://ossf-criticality-score/ruby_top_200.csv
  gs://ossf-criticality-score/rust_top_200.csv
  gs://ossf-criticality-score/shell_top_200.csv
  ```

  目前这些列表都仅仅源自github上托管的项目，计划未来扩展到其他源码控制系统上的项目。

  这些数据使用本仓库py脚本https://github.com/ossf/criticality_score/blob/main/criticality_score/generate.py来生成

  ```
  $ pip3 install python-gitlab PyGithub
  $ python3 -u -m criticality_score.generate \
      --language c --count 200 --sample-size 5000 --output-dir output
  ```

  我们还在 GitHub（独立于语言）中汇总了超过 10 万个存储库的结果，可在[此处](https://www.googleapis.com/download/storage/v1/b/ossf-criticality-score/o/all.csv?generation=1614554714813772&alt=media)下载。

​		得到一个all.csv表格

​		![image-20211129114136285](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题之四：criticality-score/image-20211129114136285.png)
