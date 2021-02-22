# 卒論のプログラム

1.各フォルダの説明
config...油圧ショベルの各ジョイントの制御方法を定めたプログラム
launch...srcの中のファイルを一度に開くためのlaunchファイル
src...操作介入を行ったり，衝突対象との衝突回数などを計測したりするプログラム
urdf...油圧ショベルのモデルを表示するプログラム
world...シミュレータ上にworldを表示するプログラム

2.srcの中の各プログラムの説明
joy2cmd...ジョイスティックの操作を油圧ショベルへの操作指令値に変換するプログラム
cmd_coorsinator_no_intervention...オペレータに対して操作介入をしない手法のプログラム
cmd_coordinator_warning_sound...衝突対象に近づいた時に警告音を鳴らす手法のプログラム（実質的には上の操作介入なしのプログラムと同じ）
cmd_coordinator_proposed...提案した操作介入手法のプログラム
cmd2ctrl...cmd_coordinatorによる介入が行われて出力された操作指令値を油圧ショベルの各ジョイントに渡すプログラム
movement_id...提案手法において，オペレータが行っている動作内容を出力するプログラム
warning_sound...警告音を鳴らす手法において，警告音を出すプログラム
そのほかのプログラムは，各衝突対象との衝突回数や，衝突リスク評価を測るためのプログラム

3.手順
ターミナルで，これらのフォルダが入っているディレクトリに移動し，roslaunch excavator_joycon_ut_vel hogeとするとシミュレータが起動し，シミュレータ上に，直方体に見立てたダンプや，人，油圧ショベルが表示される．ここで，介入なしの手法を実行する場合はhogeにexcavator_joycon_ut_no_interventionを，警告音による手法を実行する場合はhogeにexcavator_joycon_ut_warning_soundを，提案した操作介入手法を実行する場合はhogeにexcavator_joycon_ut_proposedを入れる．さらに，提案手法の場合は，別のターミナルウィンドウでrosrun excavator_joycon_ut_vel movement_idとすれば，動作内容がそのウィンドウ上に表示される
