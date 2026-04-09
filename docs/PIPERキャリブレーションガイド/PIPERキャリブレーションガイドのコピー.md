# PIPERキャリブレーションガイド

2026 /4/8 大塚 崇



## 目的

ベースのわずかなずれや関節角の誤差は、先端まで届くほど運動学的に増幅するため、環境差は拠点間で再現性のばらつきとして現れます。
ここで参照姿勢を合わせてゼロリセットすることで、リーダー／フォロワーのテレオペや DPEX 推論など、以降の作業を同じ前提で確認できるようにするためのものです。

## 1. アーム以外の環境確認



### 俯瞰カメラフレーム位置の確認

<img src="/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408071728522.png" alt="image-20260408071728522" style="zoom:50%;" />

テーブルを端からアルミフレームのテーブル側までが20cmか確認

**（※以前のPIPER環境構築ガイドでは15cmになっていたため注意)**



## ベースマウントフレームの固定確認



ベースマウントを固定するフレームは図のようにできるだけ隙間がないように固定してください。

![image-20260408072500486](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408072500486.png)



### ベースマウントフレーム固定（ダメなケース）![image-20260408072723734](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408072723734.png)



### ベースマウントフレーム固定(左右センタリング)

ベースマウントが、フレームの端から12cmずつはなれてセンターにあるか確認してください。

![image-20260408073415896](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408073415896.png)





## 2.キャリブレーション





### テレオペ設定を無効にする



テレオペが有効だとリーダーが動いてしまうため無効にします。




![](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408085901287.png)



### ティーティングモード

アームを動かすにはアームのボタンを押して、ティーチングモードに移行します。



<img src="/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409053701929.png" alt="image-20260409053701929" style="zoom:50%;" />









## キャリブレーションのイメージ

キャリブレーションの原点は以下のイメージです。この形に近づけてください。



![image-20260407231601763](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260407231601763.png)



## 各軸の調整

ジョイント１から６までの位置を調整します


###  Joint1

![image-20260408090524059](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260408090524059.png)

### Joint2



一番下げた状態を維持。

![image-20260409080615222](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409080917712.png)

### Joint3

一番下に下げた状態を維持。

![image-20260409081157140](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409081157140.png)

### Joint4

右から見て、切り欠きの位置を合わせる

![image-20260409081428914](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409081428914.png)



### Joint5

Joint5は基準になる場所が分かりにくいので注意。

図のように切り欠きとネジの位置を合わせます。

![image-20260409082303217](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409082303217.png)



### Joint6

上からみて切り欠きを合わせます。

![image-20260409081848579](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409081848579.png)

### グリッパー

グリッパーはリーダー、フォロワーともに閉じた状態にする。

![image-20260409082447697](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409082447697.png)

![image-20260409082438154](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409082438154.png)



## ゼロリセットを行う

### vlaborダッシュボードから詳細設定を開く



![image-20260409083240298](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409083240298.png)

#### 詳細設定画面を開く

詳細設定画面を開いて準備します。

以下の数値に注目します。

![image-20260409083828505](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409083828505.png)



### ティーチングモードを終了



ボタンを押して、ティーチングモードを終了し緑色のLEDが消えることを確認します。
SetZeroボタンを押すと一瞬アームが脱力するので、手で支えた状態を維持します。



![image-20260409083923692](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409083923692.png)



#### ゼロリセット

ゼロリセットを押すと一瞬脱力するので、**アームを抑えたまま作業を進めてください。**


![image-20260409084106383](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409084106383.png)

![image-20260409084204135](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409084204135.png)



Confirmボタンを押して、ゼロリセットしてください。



**※通信の読み込みと書き込みのタイミングによって１度で成功しないことがあります、その場合は何度かやってみてください。**



#### ゼロリセットの確認

完了するとすべてが０になります。ティーティングボタンを押して、原点位置にもどして０近くになっていたらOKです。

![image-20260409084234171](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409084234171.png)



## 確認

リーダー、フォロワーともに、キャリブレーションしたら、テレオペが問題ないことを確認します。

その後、DPEXの推論から Pi05-Week19-Morikawa-All-20kで推論を確認してください。



![image-20260409085845255](/Users/takatronix/projects/daihen-physical-ai/ros2_ws/src/fluent_vision_ros2/docs/PIPERキャリブレーションガイド/assets/image-20260409085845255.png)
