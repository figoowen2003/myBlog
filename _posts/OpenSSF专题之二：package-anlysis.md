---
title: OpenSSF专题之二：package-anlysis
date: 2021-11-26 16:55:22
tags:
---

# 简介

这个repo包含了一些组件来帮助分析开源包，尤其是寻找恶意软件。此代码旨在同Package Feeds项目一起使用，并且也是源自feeds项目。package feeds项目用于监控不同包仓库的变化，并发布数据到外部服务以便进一步处理。google_cloud_run_service

本repo的组件包括

- 一个调度器（scheduler）：基于来自package feeds的新的包数据来调度分析作业，这些作业会被分配给分析工作者。调度器代码是一个golang的app运行在kubernetes上，并通过ko（easy go container一个简单快捷的Go应用的容器镜像构建器，https://github.com/google/ko）部署。目前它运行在一个GKE集群上。它的目的是创建一个Pods，任何时候一个新包ready，这个Pods就会被用于分析。目前只支持pypi, npmjs, rubyGems

- 分析器（一次性分析和worker）:通过对每个包的静态和动态分析来收集包的行为数据
- 加载器（loader）：将分析结果推送到BigQuery

# 基本流程

docker-compose部署：package-feeds->scheduler->anlysis->loader

依赖于kafka(分布式发布-订阅消息系统)

在docker-compose.yml文件中配置

- packages-feeds是ossf开发的代码仓，一方面，它用来监控不同包仓库的变化，另一方面它将发布这些变化数据给package-analysis仓库进行深入分析。feeds以pipy的仓库为例（https://github.com/ossf/package-feeds/blob/main/pkg/feeds/pypi/pypi.go），通过baseURL:  "https://pypi.org/"，利用模块"net/http"去抓数据
- package-analysis仓库中的组件scheduler通过订阅topic来从feeds中接收数据，然后发送另一个topic将数据分发给不同的worker。需要在配置文件中来设置（https://github.com/ossf/package-analysis/blob/main/cmd/scheduler/config/deployment.yaml）
- worker模块会订阅scheduler的topic，接收数据，调用analysis模块（https://github.com/ossf/package-analysis/tree/main/internal/analysis）去分析数据和上传结果，其中使用了"github.com/google/gopacket"和"github.com/google/gopacket/pcap"

### 分析

`OSSMALWARE_WORKER_SUBSCRIPTION`- 可用于设置来自调度程序的数据的订阅 URL。值应遵循 [goclouddev subscriptions](https://gocloud.dev/howto/pubsub/subscribe/)。

`OSSF_MALWARE_ANALYSIS_RESULTS`-**可选**：可用于设置要将结果发布到的存储桶 URL。值应遵循 [goclouddev buckets](https://gocloud.dev/howto/blob/)。

`OSSMALWARE_DOCSTORE_URL`-**可选**：可用于设置将结果发布到的 docstore URL。值应遵循 [goclouddev docstore](https://gocloud.dev/howto/docstore/)。

### 调度器

`OSSMALWARE_WORKER_TOPIC`- 可用于设置主题 URL 以发布供分析工作者使用的数据。值应遵循 [goclouddev 发布](https://gocloud.dev/howto/pubsub/publish/)。

`OSSMALWARE_SUBSCRIPTION_URL`- 可用于设置来自[package-feeds](https://github.com/ossf/package-feeds)的数据的订阅 URL 。值应遵循 [goclouddev subscriptions](https://gocloud.dev/howto/pubsub/subscribe/)。

# 使用方法

## 本地分析

使用docker镜像gcr.io/ossf-malware-analysis/analysis，仓库中提供了脚本完成镜像编译./build/build_docker.sh。

这个容器使用了podman去运行一个嵌套的沙盒（gVisitor，是一个容器的应用程序内核，可以在任何地方提供高效的纵深防御）的容器来进行分析。

以下命令会将JSON 结果转出到/tmp/results

**实时package**，比如pypi.org上的“Django”包

```
$ mkdir /tmp/results
$ docker run --privileged -ti \
    -v /tmp/results:/results \
    gcr.io/ossf-malware-analysis/analysis分析\
    -package Django -ecosystem pypi \
    -上传文件:///结果/
```

本地package，如/path/to/test.whl，需要将包mount到容器中

```
$ mkdir /tmp/results
$ docker run --privileged -ti \
    -v /tmp/results:/results \
    -v /path/to/test.whl:/test.whl \
    gcr.io/ossf-malware-analysis/analysis分析\
    -local /test.whl -ecosystem pypi \
    -上传文件:///结果/
```

