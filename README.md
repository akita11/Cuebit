# Cue:bit

<img src="https://github.com/akita11/Cuebit/blob/main/Cuebit1.jpg" width="240px">

<img src="https://github.com/akita11/Cuebit/blob/main/Cuebit2.jpg" width="240px">

<img src="https://github.com/akita11/Cuebit/blob/main/Cuebit3.jpg" width="240px">

[使い方・動作の例(YouTube動画)](https://youtu.be/czKjS_mCOlM)


## 使い方（ライントレースモード）

電源を入れると、ライントレースを開始します。また途中にカラーマーカーがあると、それに応じた動作をします。なおライン（黒）やカラーマーカを見失って0,5秒程度経過すると、自動的に停止します（本体LEDが黄色点灯）。ラインやマーカのあるところに移動させると、動作を再開します。


### カラーマーカーと動作の定義

赤・緑・黄の3色のマーカを3つ連続して配置されたところを通過すると、その色パターンに応じた動作を行います。色マーカは10mm程度の「丸ポチシール」（DAISOなどで売っているもの）や、同等の色を印刷したものを使います。(赤=R／緑=G／黄=Y）

- 緑-赤-黄 (GRY) : 速度を超ゆっくりに
- 赤-緑-黄 (RGY) : 速度をゆっくりに
- 赤-緑-赤 (RGR) : 速度を通常に
- 黄-緑-赤 (YGR) : 速度を速く
- 黄-赤-緑 (YRG) : 速度を超速く
- 黄-赤-黄 (YRY) : 一時停止（3秒程度）
- 緑-黄-赤 (GYR) : 次の交差点を左に90度曲がる
- 黄-緑-黄 (YGY) : 次の交差点を直進する
- 緑-黄-緑 (GYG) : 次の交差点を右に90度曲がる
- 赤-黄-赤 (RYR) : Uターンする
- 緑-赤-緑 (GRG) : バックする


## 使い方（micro:bitモード）

micro:bitのカスタムブロックを使ってCue:bitの動作を制御でききます。
[こちらの手順](https://docs.google.com/document/d/1bRiQpdVX2RdSoQZLEZQx14lWcuVc2Sd8CT0sPL3RYwU/)でMakeCode用カスタムブロックを追加し、micro:bitのプログラムを作成してmicro:bitに書き込みます。その後、そのmicro:bitをCue:bitに差し込んでCue:bitの電源をいれると、そのプログラム通りにCue:bitが動作します。


### 参考：技術的仕様

micro:bitからCue:bitへのコマンド（通信プロトコル:UART 9600bps N81）は以下です。以下の"NNN"は数値(10進数)で与えるコマンドパラメータです。また各コマンドは行末コード(0x0d or 0x0a)で終端します。なおCue:bitのGroveコネクタと本体後部の6ピンコネクタ用端子(FTDI-Basic用)も、micro:bitの通信端子と並列に接続されています。

- $ : micro:bit制御モードに開始
- # : micro:bit制御モードを終了
- RNNN : NNN度回転（NNNが正=時計回り、負=反時計回り）
- B : 停止
- FNNN : NNN[cm]進む
- ZNNN : ジグザグ走行（およそNNN[cm]）
- SNNN : スケート走行（およそNNN[cm]）


以下はデバッグ用のコマンド

- P : パラメータの表示
- T : ライントレースモードを開始
- t : ライントレースモードを終了して停止
- D : デバッグ情報の表示を開始（Dに続けてr/g/y/wを書くと、色センサの値を各色の学習用のCSV形式で出力）
- d : デバッグ表示の表示を停止
- kNNN : ライントレース制御のKpパラメータ
- KNNN : ライントレース制御のKdパラメータ
- rNNN : 左右モータの速度調整（PWMデューティー比）
- vNNN : 通常走行時の速度
- fNNN : 1cm進むための時間[単位は10ms]
- gNNN : 10度回転するための時間[単位は10ms]
- ! : 各種パラメータをEEPROMに保存（起動時に読み込まれる）


## Author

Junichi Akita (@akita11) / akita@ifdl.jp
