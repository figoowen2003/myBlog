---
title: OpenSSF专题之一：scorecard
date: 2021-11-26 10:06:04
tags: go
---

# 概况

1. 什么是**Scorecard**

   Scorecard的全称为Security Scorecard，顾名思义为安全记分卡，它是一种自动化工具，提供了与软件安全相关的许多重要的启发式/探索式方法（一系列检查），并为每个检查分配0-10分。

   优势：

   ​		可以通过这些分数来了解需要改进的特定领域，以加强项目的安全状况。

   ​		可以评估引入依赖的风险，并且就究竟是接受这些风险、或者评估替代解决方案、还是与维护者合作进行改进做出合理的决定。

   它为开源工程的使用者提供了一个简单的方法去判断工程的依赖是否安全。

2. 目的

   - 实现开源项目安全状况的自动分析和信任决策
   - 使用这些数据去主动提升全世界所依赖的关键项目的安全状况

3. 主要的使用者

   目前已经有数千个项目在使用积分卡来监控和跟踪安全指标，其中比较突出的有：

   - sos.dev：Linux基金会运行的项目，为开发人员提供经济上的激励，提高关键开源项目的安全性，最初由google开源团队赞助
     - 软件供应链安全提升，比如强化CI/CD管道和分发的基础设施
     - 采用软件工件签名和验证
     - 能产生更高openSSF scorecard结果的项目改进
   - deps.dev：Open Source Insights是谷歌开发和托管的实验性服务，旨在帮助开发人员更好地了解开源软件包的结构、构造和安全性。这个服务会检查每个包，构建其依赖关系以及属性的完整且详细的图标，目标是让开发人员了解他们的软件如何组合在一起，又是如何随依赖关系的改变而变化，以及可能产生的后果。有对pypi的scorecard评分https://deps.dev/pypi/scorecard，也有go的https://deps.dev/go/github.com%2Fossf%2Fscorecard%2Fv3
   - metrics.openssf.org：OpenSSF安全度量项目，为了收集、汇总、分析和交流开源项目的安全数据。

# 使用评分卡

## 前置条件

平台：当前只支持OSX与Linux

语言：需要安装Golang

## 安装方法

- 访问最新的版本页面，下载适合操作系统的二进制文件https://github.com/ossf/scorecard/releases/latest
- 提取二进制文件
- 将二进制文件添加到GOPATH/bin目录中（必要时使用go env GOPATH去标识你的目录）

## 授权

运行记分卡之前，需要

- 创建Github的访问令牌，并且将其设置在名为GITHUB_AUTH_TOKEN`， `GITHUB_TOKEN`，`GH_AUTH_TOKEN`或`GH_TOKEN的环境变量中。这有助于防止github对未验证身份的api速率限制

  ```
  # For posix platforms, e.g. linux, mac:
  export GITHUB_AUTH_TOKEN=<your access token>
  # Multiple tokens can be provided separated by comma to be utilized
  # in a round robin fashion.
  export GITHUB_AUTH_TOKEN=<your access token1>,<your access token2>
  
  # For windows:
  set GITHUB_AUTH_TOKEN=<your access token>
  set GITHUB_AUTH_TOKEN=<your access token1>,<your access token2>
  ```

- 安装github app以获取更高的速率限制配额。如果已经安装github应用程序和密钥文件，可以使用以下三个环境变量，按照平台显示的命令进行操作。

  ```
  GITHUB_APP_KEY_PATH=<path to the key file on disk>
  GITHUB_APP_INSTALLATION_ID=<installation id>
  GITHUB_APP_ID=<app id>
  ```

## 基本用法

**docker**

记分卡以docker容器的形式使用：

环境变量GITHUB_AUTH_TOKEN必须设置为一个有效的令牌（token）

```
docker run -e GITHUB_AUTH_TOKEN=token gcr.io/openssf/scorecard:stable --show-details --repo=https://github.com/ossf/scorecard
```

**使用仓库的URL**

记分卡运行时，可以只使用一个参数-目标仓库的URL:

```
$ scorecard --repo=github.com/ossf-tests/scorecard-check-branch-protection-e2e
Starting [CII-Best-Practices]
Starting [Fuzzing]
Starting [Pinned-Dependencies]
Starting [CI-Tests]
Starting [Maintained]
Starting [Packaging]
Starting [SAST]
Starting [Dependency-Update-Tool]
Starting [Token-Permissions]
Starting [Security-Policy]
Starting [Signed-Releases]
Starting [Binary-Artifacts]
Starting [Branch-Protection]
Starting [Code-Review]
Starting [Contributors]
Starting [Vulnerabilities]
Finished [CI-Tests]
Finished [Maintained]
Finished [Packaging]
Finished [SAST]
Finished [Signed-Releases]
Finished [Binary-Artifacts]
Finished [Branch-Protection]
Finished [Code-Review]
Finished [Contributors]
Finished [Dependency-Update-Tool]
Finished [Token-Permissions]
Finished [Security-Policy]
Finished [Vulnerabilities]
Finished [CII-Best-Practices]
Finished [Fuzzing]
Finished [Pinned-Dependencies]

RESULTS
-------
Aggregate score: 7.9 / 10

Check scores:
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
|  SCORE  |          NAME          |             REASON             |                         DOCUMENTATION/REMEDIATION                         |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 10 / 10 | Binary-Artifacts       | no binaries found in the repo  | github.com/ossf/scorecard/blob/main/docs/checks.md#binary-artifacts       |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 9 / 10  | Branch-Protection      | branch protection is not       | github.com/ossf/scorecard/blob/main/docs/checks.md#branch-protection      |
|         |                        | maximal on development and all |                                                                           |
|         |                        | release branches               |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| ?       | CI-Tests               | no pull request found          | github.com/ossf/scorecard/blob/main/docs/checks.md#ci-tests               |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | CII-Best-Practices     | no badge found                 | github.com/ossf/scorecard/blob/main/docs/checks.md#cii-best-practices     |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 10 / 10 | Code-Review            | branch protection for default  | github.com/ossf/scorecard/blob/main/docs/checks.md#code-review            |
|         |                        | branch is enabled              |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | Contributors           | 0 different companies found -- | github.com/ossf/scorecard/blob/main/docs/checks.md#contributors           |
|         |                        | score normalized to 0          |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | Dependency-Update-Tool | no update tool detected        | github.com/ossf/scorecard/blob/main/docs/checks.md#dependency-update-tool |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | Fuzzing                | project is not fuzzed in       | github.com/ossf/scorecard/blob/main/docs/checks.md#fuzzing                |
|         |                        | OSS-Fuzz                       |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 1 / 10  | Maintained             | 2 commit(s) found in the last  | github.com/ossf/scorecard/blob/main/docs/checks.md#maintained             |
|         |                        | 90 days -- score normalized to |                                                                           |
|         |                        | 1                              |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| ?       | Packaging              | no published package detected  | github.com/ossf/scorecard/blob/main/docs/checks.md#packaging              |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 8 / 10  | Pinned-Dependencies    | unpinned dependencies detected | github.com/ossf/scorecard/blob/main/docs/checks.md#pinned-dependencies    |
|         |                        | -- score normalized to 8       |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | SAST                   | no SAST tool detected          | github.com/ossf/scorecard/blob/main/docs/checks.md#sast                   |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 0 / 10  | Security-Policy        | security policy file not       | github.com/ossf/scorecard/blob/main/docs/checks.md#security-policy        |
|         |                        | detected                       |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| ?       | Signed-Releases        | no releases found              | github.com/ossf/scorecard/blob/main/docs/checks.md#signed-releases        |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 10 / 10 | Token-Permissions      | tokens are read-only in GitHub | github.com/ossf/scorecard/blob/main/docs/checks.md#token-permissions      |
|         |                        | workflows                      |                                                                           |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|
| 10 / 10 | Vulnerabilities        | no vulnerabilities detected    | github.com/ossf/scorecard/blob/main/docs/checks.md#vulnerabilities        |
|---------|------------------------|--------------------------------|---------------------------------------------------------------------------|

```

**计分标准**

每个单独的检查都会返回 0 到 10 的分数，其中 10 代表可能的最佳分数。记分卡还会产生一个总分，这是按风险加权的单个检查的基于权重的平均值。

- “关键”风险检查的权重为 10
- “高”风险检查的权重为 7.5
- “中等”风险检查的权重为 5
- “低”风险检查的权重为 2.5

注意：目前没有评分卡检查被评为“严重”风险。

被评为“高”风险的测试是：

- 保持
- 依赖更新工具
- 二进制工件
- 分支保护
- 代码审查
- 签名版本
- 令牌权限
- 漏洞

被评为“中等”风险的测试是：

- 模糊测试
- 包装
- 固定依赖
- 第一天
- 安全策略

被评为“低”风险的测试是：

- CI-测试
- CII-最佳实践
- 贡献者

## 检查Checks

针对目标项目，以下的检查都会默认运行：

| Name                   | Description                                                  |
| ---------------------- | ------------------------------------------------------------ |
| Binary-Artifacts       | Is the project free of checked-in binaries? 在项目的仓库中是否生成了可执行二进制文件，用户可能会直接使用他们，导致危险行为。因为，无法审查二进制文件，可能包含而已代码；还可能导致无法创建可执行文件。在代码"github.com/ossf/scorecard/v3/checks/raw"中完成。 |
| Branch-Protection      | Does the project use [Branch Protection](https://docs.github.com/en/free-pro-team@latest/github/administering-a-repository/about-protected-branches) ? 检查项目的默认分支是否收到github的分支保护设置的保护。https://github.com/ossf/scorecard/blob/main/checks/branch_protection.go中实现 |
| CI-Tests               | Does the project run tests in CI, e.g. [GitHub Actions](https://docs.github.com/en/free-pro-team@latest/actions), [Prow](https://github.com/kubernetes/test-infra/tree/master/prow)? 检查确定项目是否在合并拉去请求之前运行CI测试，目前仅限github上的仓库。检查github CheckRuns和Statuses最近的提交记录中（30次以内包含30次）一组CI系统的名称集合：appveyor、buildkite、circleci、e2e、github-actions、jenkins、mergeable、test、travis-ci  https://github.com/ossf/scorecard/blob/main/checks/ci_tests.go |
| CII-Best-Practices     | Does the project have a [CII Best Practices Badge](https://bestpractices.coreinfrastructure.org/en)? 检查项目是否获得了CII最佳实践徽章，这个徽章表明该项目使用了一组以安全为中心的开源软件最佳开发实践。使用了Git仓库了URL和CII API。https://github.com/ossf/scorecard/blob/main/checks/cii_best_practices.go |
| Code-Review            | Does the project require code review before code is merged? 检查项目在合并pr之前是否需要代码审查。首先尝试检查是否在默认分支上启用了分支保护，提至少需要一个审查者。如果失败了，则检查最近（<=30）次提交是否有github所允许的审查，或者合并是否与提交者不同。还需要执行prow(https://github.com/kubernetes/test-infra/tree/master/prow#readme)这个k8s的CI/CD系统和Gerrit Code Review（谷歌开发的review工具）https://github.com/ossf/scorecard/blob/main/checks/code_review.go |
| Contributors           | Does the project have contributors from at least two different organizations?  检查项目是否有来自多个组织的的近期贡献者。仅限于github上的仓库。会查看项目最近提交者的github个人资料的company字段，要获得最高分，项目必须在最近 30 次提交中至少有来自 3 家不同公司的贡献者；在过去的 30 次提交中，每个贡献者必须至少有 5 次提交。https://github.com/ossf/scorecard/blob/main/checks/contributors.go |
| Dependency-Update-Tool | Does the project use tools to help update its dependencies? 检查项目是或否使用依赖更新工具，尤其是dependabot或renovatebot。只能判断是否开启了依赖更新工具 https://github.com/ossf/scorecard/blob/main/checks/dependency_update_tool.go |
| Fuzzing                | Does the project use fuzzing tools, e.g. [OSS-Fuzz](https://github.com/google/oss-fuzz)? 检查是否使用了模糊测试。 https://github.com/ossf/scorecard/blob/main/checks/fuzzing.go |
| Maintained             | Is the project maintained? 检查项目是否处于主动维护状态，如果已经存档了，则得分最低。如果在过去 90 天内每周至少有一次提交，则该项目获得最高分。https://github.com/ossf/scorecard/blob/main/checks/maintained.go |
| Pinned-Dependencies    | Does the project declare and pin [dependencies](https://docs.github.com/en/free-pro-team@latest/github/visualizing-repository-data-with-graphs/about-the-dependency-graph#supported-package-ecosystems)?  检查项目是不是一个已经声明并且固定了依赖项的应用程序。固定依赖项试制明确设置为特定版本而不是允许适配一个版本范围。需要检查 根目录下的以下文件：go.mod, go.sum (Golang), package-lock.json, npm-shrinkwrap.json (Javascript), requirements.txt, pipfile.lock (Python), gemfile.lock (Ruby) )、cargo.lock (Rust)、yarn.lock (包管理器)、composer.lock (PHP)、vendor/、third_party/、第三方/；Dockerfiles、shell 脚本和 GitHub 工作流中的非固定依赖项。https://github.com/ossf/scorecard/blob/main/checks/pinned_dependencies.go |
| Packaging              | Does the project build and publish official packages from CI/CD, e.g. [GitHub Publishing](https://docs.github.com/en/free-pro-team@latest/actions/guides/about-packaging-with-github-actions#workflows-for-publishing-packages) ? 检查项目是否被发布为一个package。检查github的打包工作流和特定语言的github操作，这些操作将包上传到相应的中心。比如Npm但是目前还不支持NPM,PyPI。https://github.com/ossf/scorecard/blob/main/checks/packaging.go |
| SAST                   | Does the project use static code analysis tools, e.g. [CodeQL](https://docs.github.com/en/free-pro-team@latest/github/finding-security-vulnerabilities-and-errors-in-your-code/enabling-code-scanning-for-a-repository#enabling-code-scanning-using-actions), [LGTM](https://lgtm.com/), [SonarCloud](https://sonarcloud.io/)? 是否使用静态程序安全测试Static Application Security Testing，也叫静态代码分析。在最近合并（30个）的pr中查找已知的github应用比如 [CodeQL](https://codeql.github.com/)（github-code-scanning）、 [LGTM](https://lgtm.com/)和 [SonarCloud](https://sonarcloud.io/)，或在 GitHub 工作流程中使用“github/codeql-action”。  https://github.com/ossf/scorecard/blob/main/checks/sast.go |
| Security-Policy        | Does the project contain a [security policy](https://docs.github.com/en/free-pro-team@latest/github/managing-security-vulnerabilities/adding-a-security-policy-to-your-repository)? 此检查尝试确定项目是否已发布安全策略。它的工作原理是``在几个众所周知的目录中查找名为SECURITY.md（不区分大小写）的文件。 https://github.com/ossf/scorecard/blob/main/checks/security_policy.go |
| Signed-Releases        | Does the project cryptographically [sign releases](https://wiki.debian.org/Creating signed GitHub releases)? 此检查尝试确定项目是否对发布工件进行加密签名，此检查在项目的最后五个版本中查找以下文件名：[*.minisig](https://github.com/jedisct1/minisign)、*.asc (pgp)、*. [sig](https://github.com/jedisct1/minisign)、*.sign。https://github.com/ossf/scorecard/blob/main/checks/signed_releases.go |
| Token-Permissions      | Does the project declare GitHub workflow tokens as [read only](https://docs.github.com/en/actions/reference/authentication-in-a-workflow)? 此检查确定项目的自动化工作流令牌是否默认设置为只读。当每个工作流的 yaml 文件中的权限定义在[顶级](https://docs.github.com/en/actions/reference/workflow-syntax-for-github-actions#permissions)设置为只读 https://github.com/ossf/scorecard/blob/cc4949465b6730ee398e49a096e0132f02078372/checks/permissions.go |
| Vulnerabilities        | Does the project have unfixed vulnerabilities? Uses the [OSV service](https://osv.dev/). 此检查使用[OSV（开源漏洞）](https://osv.dev/)服务（提供了开源漏洞的数据库）确定项目是否具有开放的、未[修复的漏洞](https://osv.dev/) https://github.com/ossf/scorecard/blob/main/checks/vulnerabilities.go  数据集是否可以下载 |

## 公共数据库

如何有兴趣查看一系列带有scorecard检查结果的项目列表，它将结果发布在BigQuery pulbic dataset这个数据集中BigQuery 视图中提供了最新结果`openssf:scorecardcron.scorecard-v2_latest`

https://cloud.google.com/bigquery/public-data

公共数据集是存储于 BigQuery ，并通过 [Google Cloud 公共数据集计划](https://cloud.google.com/public-datasets)提供给公众的任何数据集。公共数据集是由 BigQuery 托管的数据集，可供您访问并集成到您的应用中。Google 会支付这些数据集的存储费用，并通过[项目](https://cloud.google.com/docs/overview#projects)提供对数据的公开访问权限。您只需为对数据执行的查询付费。每月免费处理前 1 TB 数据，具体参阅[查询价格详情](https://cloud.google.com/bigquery/pricing#analysis_pricing_models)。

您可以使用旧版 SQL 或[标准 SQL](https://cloud.google.com/bigquery/docs/reference/standard-sql/query-syntax) 查询对公共数据集进行分析。查询公共数据集时，请使用完全限定的表名称，例如 `bigquery-public-data.bbc_news.fulltext`。

您可以通过以下方式访问 BigQuery 公共数据集：使用 [Cloud Console](https://console.cloud.google.com/marketplace/partners/bigquery-public-data)、使用 [`bq` 命令行工具](https://cloud.google.com/bigquery/docs/cli_tool)，或者使用各种[客户端库](https://cloud.google.com/bigquery/docs/reference/libraries)（例如[Java](https://developers.google.com/api-client-library/java/apis/bigquery/v2)、[.NET](https://developers.google.com/api-client-library/dotnet/get_started) 或 [Python](https://developers.google.com/api-client-library/python/)）调用 [BigQuery REST API](https://cloud.google.com/bigquery/docs/reference/v2)。

## 如何一个其他的github仓库让在scorecard的dailyscore去扫描

scorecard维护了一个仓库的列表文件https://github.com/ossf/scorecard/blob/main/cron/projects.txt，提交一个PR去修改这个文件，那么scorecard会在下次运行时启动扫描

https://github.com/ossf/scorecard/blob/cc4949465b6730ee398e49a096e0132f02078372/CONTRIBUTING.md#where-the-ci-tests-are-configured
