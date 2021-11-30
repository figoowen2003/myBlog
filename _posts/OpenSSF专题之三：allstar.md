---
title: OpenSSF专题之三：allstar
date: 2021-11-27 16:02:17
tags:
---

# 概念

Allstar是一个安装在组织（organization）或者仓库（repositories）上的Github App。它的目的是能够持续监控和检测任何有风险或者不符合安全最佳实践的Github设置或者仓库文件。

如果Allstar发现仓库不合规，它将采取诸如创建issue或者恢复安全设置的措施。

- Allstar的具体策略是高度可配置的，以满足不同项目社区和组织的需求
- 指定和贡献新的政策也很容易



# 快速开始

在组织或者个人的github仓库上快速安装Allstar app，连接为https://github.com/apps/allstar-app，网页式安装。安装Allstar时，用户需要审核app请求的权限。Allstar app要求对大多数的设置和文件内容具有度权限，以检测安全合规性。它还会申请对issue的写权限以创建issue，对checks的写权限以允许阻塞的动作。（具体权限如https://github.com/ossf/allstar/blob/main/operator.md）

## 快速配置

前提：已经在我的账号/组织上安装了Allstar

默认的，你的账号/组织上的Allstar不会采取任何动作。需要进行使能配置

- 快速使能Allstar

  1. 创建一个名为.allstar的仓库。

  2. 在上述仓库中创建一个名为allstar.yaml的配置文件，写入以下内容

     ```
     optConfig:
       optOutStrategy: true
     ```

  3. 创建四个文件`branch_protection.yaml`， `binary_artifacts.yaml`，`outside.yaml`，和`security.yaml`，并且都写入以下内容

     ```
     optConfig:
       optOutStrategy: true
     action: issue
     ```

  这样，我们就在所有仓库中使能了Allstar及其所有的策略policies（默认配置）.issue动作将在每一个违背了安全策略的仓库中创建github issue。

  如果仅仅希望使能一部分的仓库，那么在allstar.yaml文件中这样配置

  ```
  optConfig:
    optInRepos:
    - repo-one
    - repo-two
  ```

  以上配置是organization级别的

  **仓库级别的配置如下：**

  1. 创建目录.allstar/

  2. 在上述目录中新建文件allstar.yaml，写入以下内容

     ```
     optConfig:
       optIn: true
     ```

  3. 创建四个文件`branch_protection.yaml`， `binary_artifacts.yaml`，`outside.yaml`，和`security.yaml`，写入以下内容

     ```
     optConfig:
       optIn: true
     action: issue
     ```

## Allstar配置策略strategy

- opt-in 只有在配置文件中显示列出的 仓库才可以使能Allstar
- opt-out 所有的仓库都可以使能Allstar，并且仓库需要显著地添加opt-out。

## 当前支持的策略policies

与 Allstar app使能配置类似，所有策略要么使用组织级的.allstar仓库中的yaml文件，要么使用仓库级别.allstar目录下的yaml文件来配置以让其使能。与应用程序一样，默认情况下是opt-in，默认`log`操作也不会产生可见的结果。启用所有策略的一种简单方法是为每个策略创建一个包含以下内容的 yaml 文件：https://github.com/ossf/allstar/tree/main/pkg/policies

```
optConfig:
  optOutStrategy: true
action: issue
```

### 分支保护

此策略的配置文件名为`branch_protection.yaml`，[配置定义在此处](https://pkg.go.dev/github.com/ossf/allstar@v0.0.0-20210728182754-005854d69ba7/pkg/policies/branch#OrgConfig)。

分支保护策略 根据指定的配置检查 GitHub 的[分支保护设置](https://docs.github.com/en/github/administering-a-repository/defining-the-mergeability-of-pull-requests/about-protected-branches)是否正确[设置](https://docs.github.com/en/github/administering-a-repository/defining-the-mergeability-of-pull-requests/about-protected-branches)。问题文本将描述哪个设置不正确。有关 更正设置，请参阅[GitHub 的文档](https://docs.github.com/en/github/administering-a-repository/defining-the-mergeability-of-pull-requests/about-protected-branches)。

### 二进制工件

此策略的配置文件名为`binary_artifacts.yaml`，[配置定义在此处](https://pkg.go.dev/github.com/ossf/allstar@v0.0.0-20210728182754-005854d69ba7/pkg/policies/binary#OrgConfig)。

此策略包含[来自记分卡](https://github.com/ossf/scorecard/#scorecard-checks)的[检查](https://github.com/ossf/scorecard/#scorecard-checks)。从存储库中删除二进制工件以实现合规性。由于记分卡结果可能很冗长，您可能需要运行[记分卡本身](https://github.com/ossf/scorecard)才能查看所有详细信息。

### 外部合作者

此策略的配置文件名为`outside.yaml`，[配置定义在此处](https://pkg.go.dev/github.com/ossf/allstar@v0.0.0-20210728182754-005854d69ba7/pkg/policies/outside#OrgConfig)。

此策略检查是否有任何[外部协作者](https://docs.github.com/en/organizations/managing-access-to-your-organizations-repositories/adding-outside-collaborators-to-repositories-in-your-organization) 对存储库具有管理员（默认）或推送（可选）访问权限。只有组织成员才能拥有此访问权限，否则不受信任的成员可以更改管理员级别设置并提交恶意代码。

### 安全文件

此策略的配置文件名为`security.yaml`，[配置定义在此处](https://pkg.go.dev/github.com/ossf/allstar@v0.0.0-20210728182754-005854d69ba7/pkg/policies/security#OrgConfig)。

此策略检查存储库中是否有安全策略文件 `SECURITY.md`并且它不为空。创建的问题将包含一个指向[GitHub 选项卡](https://docs.github.com/en/code-security/getting-started/adding-a-security-policy-to-your-repository)的链接 ，可帮助您向存储库提交安全策略。

## Action

每个策略policy都可以配置一个 Allstar 的操作，当检测到存储库不合规时。

- `log`：这是默认操作，实际上对所有操作都会发生。记录所有策略运行结果和详细信息。日志目前仅对应用程序运营商可见，公开这些的计划正在讨论中。
- `issue`：此操作会导致 GitHub 问题。每个策略只创建一个问题，文本描述了策略违规的详细信息。如果问题已经打开，则会每 24 小时用一条评论对其进行 ping 测试（当前用户不可配置）。一旦违规得到解决，Allstar 将在 5-10 分钟内自动关闭该问题。
- `fix`：此操作是特定于政策的。该策略将对 GitHub 设置进行更改以更正策略违规。并非所有政策都能支持这一点（见下文）。

已提议但尚未实施的行动。将来会添加定义。

- `block`：Allstar 可以设置[GitHub 状态检查](https://docs.github.com/en/github/collaborating-with-pull-requests/collaborating-on-repositories-with-code-quality-features/about-status-checks) 并在检查失败时阻止存储库中的任何 PR 被合并。
- `email`：Allstar 会向存储库管理员发送电子邮件。
- `rpc`：Allstar 会向某些特定于组织的系统发送 rpc。

# 创建自己的Allstar实例app

https://github.com/ossf/allstar/blob/main/operator.md

- 首先是创建一个github app 参考https://docs.github.com/en/developers/apps/building-github-apps/creating-a-github-app

  - **Name/Description/Homepage URL** Something specific to your instance.

  - **Callback URL** Leave blank, Allstar does not auth as a user.

  - **Request user authorization (OAuth) during installation** uncheck.

  - **Webhooks/Subscribe to events** Uncheck and leave blank. Allstar does not listen for webhooks at this time.

  - **Permissions** Follow this example: 

    ![image](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题之三：allstar/121067612-1bbc5200-c780-11eb-9bd3-214dfe808bf7.png)

- 获取ID和key 参考https://docs.github.com/en/developers/apps/building-github-apps/authenticating-with-github-apps

  还要记下新应用程序的“常规”或“关于”部分中的应用程序 ID。

  通过[Go CDK 运行时配置](https://gocloud.dev/howto/runtimevar/)将私钥内容上传到支持的服务

  编辑`pkg/config/operator/operator.go`并设置 AppID 和 KeySecret 链接。您可能需要`pkg/ghclients/ghclients.go`为您的秘密服务编辑和添加新的导入行，例如：`_ "gocloud.dev/runtimevar/gcpsecretmanager"`。

- 运行Allstar

- `cmd/allstar/`（https://github.com/ossf/allstar/tree/main/cmd/allstar）在任何环境中构建和运行。不需要 cli 配置。Allstar 当前不侦听 webhooks，因此不需要传入网络配置。仅向 GitHub 发出呼叫。最好只运行一个实例，以避免在强制措施中出现潜在的竞争条件，例如：同时 ping 一个问题两次。
