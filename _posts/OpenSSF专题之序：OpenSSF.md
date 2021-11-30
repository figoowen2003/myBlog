---
title: OpenSSF专题之序：OpenSSF
date: 2021-11-29 14:36:22
tags: https://github.com/ossf/foundation
---

# 开源安全基金会Open Source Security Foundation - OpenSSF

**共同守护开源生态系统**

开源软件已经在数据中心、消费设备和服务中日益普及，它的价值愈发体现在技术人员和企业中。由于其开发过程的影响，最终到达端侧用户的 开源软件会具有一系列贡献者和依赖项。对用软件用户或者组织的安全性而言，理解并确认依赖链的安全非常重要。

# 技术举措

## 漏洞披露vulnerability Disclosure

OpenSSF漏洞披露工作组旨在通过辅助开发和倡导管理良好的漏洞报告和通信机制来提高开源生态系统的整体安全性。

- 提供文档化的标准和培训材料，在OSS生态系统内为组件维护者和社区成员记录并促进合理的漏洞披露和协调实践。
- 明确OSS维护者、使用者和安全研究人员的漏洞披露痛点，并逐步解决这些问题。
- 促进基于标准的OSS漏洞交换协议的开发和应用，该交换协议使用现有的行业格式，并允许所有规模的OSS项目都能报告、共享和了解OSS组件中的漏洞

当前工作有

- [开源软件项目协同漏洞披露指南，](https://github.com/ossf/oss-vulnerability-guide)协助项目处理漏洞。
- [开源漏洞架构](https://github.com/ossf/osv-schema)- 另见[osv.dev](https://osv.dev/)。
- [OpenSSF Recommendations for Open Source Software Vulnerability Disclosure Whitepaper](https://docs.google.com/document/d/1ggvl7_p7-tmieP5He1dSmRbndDz1CG2_BqNpk6ss6ks/edit) (incubating) - 关于各种相关主题的较长论文草案
- [漏洞报告和披露的统一元数据列表](https://docs.google.com/spreadsheets/d/1eZpBk2aIup29KcWwN5MAhvkk60EE_DRt2YRtLo8P0zs/edit?usp=sharing)

## 安全工具 Security Tooling

帮助开发者扫除安全缺陷，让他们能够专注与所需要的特性开发。

- 识别 - 开发人员可以在各种开发环境中使用大量工具。我们需要确保我们了解可用的选项。
- 评估 - 有些工具比其他工具更好。我们需要确保开源社区可以使用高质量的工具。
- 改进 - 某些工具只需要一点点帮助即可提供最佳解决方案。在可能的情况下，我们需要为改进这些工具做出贡献。
- 开发——尽管有大量可用的工具，但仍有很大一部分安全问题空间没有帮助开发人员发现问题的工具。我们将开发那些有兴趣和带宽的工具。
- 轻松部署 -**最重要的是**，开源开发人员需要知道他们应该使用哪些工具以及如何轻松地将它们集成到他们的开发过程中。除非开发人员有一种简单的方法来加入安全工具，否则它不太可能被包含在内。我们将向开源开发人员提供这些信息。

## 最佳安全实践 Security Best Practices

目标是为开源开发人员提供最佳实践建议，并提供一种学习和应用它们的简单方法。

我们的愿景是让开发人员轻松采用这些最佳实践，这要归功于：

- *确定*有助于开源开发人员创建和维护更安全软件的良好实践、要求和工具
- 帮助维护者*学习*编写安全软件
- 提供工具，以帮助开发人员*采用*这些好的做法纳入其日常工作

我们的工作被组织成几个离散但相关的项目，帮助我们实现我们的目标：

- *Common Requirement Enumeration (CRE) Project* - (incubating) https://github.com/OWASP/www-project-integration-standards
  - 目的——（识别）识别不同规范中的相似要求
- *安全软件开发基础*（edX 课程） - https://openssf.org/training/courses/和https://github.com/ossf/secure-sw-dev-fundamentals
  - 目的 -（学习）教授软件开发人员开发安全软件的基础知识
- *SKF - 安全知识框架*- https://www.securityknowledgeframework.org/
  - 目的 -（识别/采用/学习）学习通过设计将安全性集成到您的 Web 应用程序中
- *CII 最佳实践徽章*- https://bestpractices.coreinfrastructure.org/和https://github.com/coreinfrastructure/best-practices-badge
  - 目的 -（识别/采用）识别 FLOSS 最佳实践并为这些实践实施标记系统，
- *记分卡项目*- https://github.com/ossf/scorecard
  - 目的 -（采用）自动化分析和信任开源项目安全态势的决策。
- *伟大的 MFA 分发项目*-（孵化）https://github.com/ossf/great-mfa-project
  - 将 MFA 令牌分发给 OSS 开发人员以及如何轻松使用它们的最佳实践
- [C/C++ 程序的推荐编译器选项标志](https://docs.google.com/document/d/1SslnJuqbFUyTFnhzkhC_Q3PPGZ1zrG89COrS6LV6pz4/edit#heading=h.b3casmpemf1b)（孵化）
  - 为 C/C++ 程序推荐的编译器选项标志，尤其是警告和强化标志，供开发人员和发行版使用

# 识别开源项目的安全威胁 Identifying Security Threats to Open Source Projects

目标是让利益相关者对开源项目的安全性有充分的信心。这包括识别对开源生态系统的威胁并推荐实用的缓解措施。我们还将确定一组关键指标并构建工具将这些指标传达给利益相关者，从而更好地了解各个开源软件组件的安全状况。

- [安全指标](https://github.com/ossf/Project-Security-Metrics)- 该项目的目的是为开源项目收集、组织和向利益相关者（包括用户）提供有益的的安全指标。[**查看已部署的安全指标仪表板网站**](https://metrics.openssf.org/)
- [安全审查](https://github.com/ossf/security-reviews)- 此存储库包含开源软件的安全审查集合。它用作安全指标的输入。
- [项目安全信息规范 (OSSF-SECURITY.yml))](https://docs.google.com/document/d/1Hqks2J0wVqS_YFUQeIyjkLneLfo3_9A-pbU-7DZpGwM/edit) - 早期草案工作，用于在项目中捕获一些与安全相关的机器可处理信息
- Alpha-Omega - Alpha-Omega 项目将是它自己的项目，但它已在该工作组内孵化

## 保护关键项目 Securing Critical Projects

大大小小的开源项目都面临着资源分配的问题，包括所需的时间、资源和注意力。需要一个将关键项目与可为其提供支持的组织联系起来的方法。

- [criticality_score](https://github.com/ossf/criticality_score) - 尝试使用[Rob Pike 的“量化关键性”中](https://github.com/ossf/criticality_score/blob/main/Quantifying_criticality_algorithm.pdf)描述的算法来估计关键性.
- [Harvard research](https://www.coreinfrastructure.org/programs/census-program-ii/)
- [package-feeds](https://github.com/ossf/package-feeds) / [package-analysis](https://github.com/ossf/package-analysis)
- [allstar](https://github.com/ossf/allstar)

## 数字身份认证 Digital identity Attestation

目标是让开源维护者、贡献者和最终用户能够理解他们维护、生产和使用的代码的出处或来源并做出决定。



- 为开源维护者提供一种以他们选择的名义开展工作的方式，并防止其他人冒充他们。
- 根据他们选择的标准，为开源社区提供工具和基础设施来验证其维护者的身份。
- 为开源库的消费者提供更多数据，以确定依赖该库的风险。
- 为消费者和维护者提供可信赖的公开记录，说明谁对开源软件项目实施了更改。
- 尊重所有相关人员的隐私。
- 使 OSS 维护人员能够更好地确保遵循项目治理策略（如独立签收）。
- 为 OSS 消费者提供工具来检测未知贡献者的活动变化。
- 允许开源消费者检查其开源供应链的完整来源。
- 允许人与人之间的信任，不需要受信任的中介。
