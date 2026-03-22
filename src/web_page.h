const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>UGV01_BASE_WEB</title>
    <meta name="viewport" content="width=device-width,initial-scale=1.0">
    <!-- <script src="http://code.jquery.com/jquery-1.9.1.min.js"></script> -->
    <style type="text/css">
    html {
        display: inline-block;
        text-align: center;
        font-family: sans-serif;
    }
    body {
        background-image: -webkit-linear-gradient(#3F424F, #1E212E);
        font-family: "roboto",helt "sans-serif";
        font-weight: lighter;
        background-position: center 0;
        background-attachment: fixed;
        color: rgba(255, 255, 255, 0.6);
        font-size: 14px;
    }
    .cc-btn {
        border: 0;
        cursor: pointer;
        color: #fff;
        background: rgba(164,169,186,0);
        font-size: 1em;
        width: 100px;
        height: 100px;
         -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none; 
    }
    .cc-middle{
        width: 100px;
        height: 100px;
        border-radius: 50%;
        background-color: rgba(94,98,112,0.8);
    }
    .cc-btn:hover svg, .cc-middle:hover {
        opacity: 0.5;
    }
    .cc-btn:active svg, .cc-middle:hover{
        opacity: 0.5;
    }
    .controlor-c > div{
        width: 300px;
        height: 300px; 
        background-color: rgba(94,98,112,0.2);
        border-radius: 40px;
        box-shadow: 10px 10px 10px rgba(0,0,0,0.05);
        margin: auto;
    }
    .controlor-c > div > div{
        display: flex;
    }
    main {
        width: 960px;
        margin: auto;
    }
    section{margin: 40px 0;}
    .for-move {
        display: flex;
        align-items: center;
    }
    .for-move-a, .for-move-b{
        flex: 1;
        margin: 0 20px;
    }
    .h2-tt {
        font-size: 2em;
        font-weight: normal;
        color: rgba(255, 255, 255, 0.8);
        text-transform: uppercase;
    }
    .info-device-box .info-box{display: flex;}
    .info-device-box .info-box{padding: 20px 0;}
    .num-box-big > div, .num-box-sma > div{flex: 1;}
    .num-box-big > div:first-child{border-right: 1px solid rgba(216,216,216,0.1);}
    .num-box-mid {
        flex-wrap: wrap;
        justify-content: space-between;
    }
    .num-box-mid div{
        width:33.3333%;
        margin: 20px 0;
    }
    .info-device-box .info-box > div > span {
        display: block;
    }
    .info-box {
        background-image: linear-gradient(to right, rgba(94, 98, 112, 0.3), rgba(75, 80, 95, 0.3)) ;
        margin: 20px auto;
        border: 1px solid rgba(216, 216, 216, 0.1);
        box-shadow: 10px 10px 10px rgba(0,0,0,0.05);
        border-radius: 4px;
        color: rgba(255,255,255,0.5);
    }
    .big-num{font-size: 3em;}
    .mid-num{font-size: 2em;}
    .sma-num{font-size: 1.2em;}
    .num-color{
        background-image: linear-gradient(rgba(255,255,255,1),rgba(255,255,255,0.5));
        background-clip: text;
        color:transparent;
        -webkit-background-clip: text;
        -moz-background-clip: text;
        -ms-background-clip: text;
        font-weight: 900;
        line-height: 1em;
        margin: 0.5em 0;
    }
    .num-color-red{
        background-image: linear-gradient(rgba(181,104,108,1),rgba(181,104,108,0.5));
        background-clip: text;
        color:transparent;
        -webkit-background-clip: text;
        -moz-background-clip: text;
        -ms-background-clip: text;
        font-weight: 900;
        line-height: 1em;
        margin: 0.5em 0;
    }
    .controlor > div {margin: 80px 0;}
    .json-cmd-info{
        display: flex;
        flex-wrap: wrap;
    }
    .json-cmd-info > div {
        width: 33.33333%;
        padding: 10px 0;
    }
    .json-cmd-info p{
        line-height: 30px;
        margin: 0;
    }
    .json-cmd-info p span {
        display: block;
        color: rgba(255,255,255,0.8);
    }
    .small-btn{
        color: rgba(255,255,255,0.8);
        background-color: #5E6270;
        border: none;
        height: 48px;
        border-radius: 4px;
    }
    .small-btn-active{
        background-color: rgba(38,152,234,0.1);
        color: #2698EA;
        border: 1px solid #2698EA;
        height: 48px;
        border-radius: 4px;
    }
    .feedb-p input{
        width: 100%;
        height: 46px;
        background-color: rgba(0,0,0,0);
        padding: 0 10px;
        border: 1px solid rgba(194,196,201,0.15);
        border-radius: 4px;
        color: rgba(255, 255, 255, 0.8);
        font-size: 1.2em;
        margin-right: 10px;
    }
    .control-speed > div {
        width: 290px;
        margin: auto;
    }
    .control-speed > div > div{display: flex;}
    .control-speed label {flex: 1;}
    .small-btn, .small-btn-active{
        width: 90px;
    }
    .feedb-p{ display: flex;}
    .fb-input-info{
        margin: 0 20px;
    }
    .fb-info {margin: 20px;}
    .fb-info > span{line-height: 2.4em;}
    .btn-send:hover, .small-btn:hover{background-color: #2698EA;}
    .btn-send:active, .small-btn:active{background-color: #1b87d4;}
    .w-btn{
        color: #2698EA;
        background: transparent;
        padding: 10px;
        border: none;
    }
    .w-btn:hover{color: #2698EA;}
    .w-btn:active{color: #1b87d4;}
    @media screen and (min-width: 768px) and (max-width: 1200px){
        body{font-size: 16px;}
        main {
            width: 100%;
        }
        .for-move {
            display: block;
        }
        /* .controlor-c > div{width: 600px;height: 600px;}
        .cc-btn{width: 200px;height: 200px;} */
        .json-cmd-info{display: block;}
        .json-cmd-info p span{display: inline;}
        .json-cmd-info > div{
            display: flex;
            width: auto;
            padding: 20px;
            flex-wrap: wrap;
            justify-content: space-between;
        }
        .control-speed > div{width: 600px;}
        section{margin: 20px 0;}
    }
    @media screen and (min-width: 360px) and (max-width: 767px){
        main {
            width: 100%;
        }
        .for-move {
            display: block;
        }
        .json-cmd-info{display: block;}
        .json-cmd-info p span{display: inline;}
        .json-cmd-info > div{
            display: flex;
            width: auto;
            padding: 20px;
            flex-wrap: wrap;
            justify-content: space-between;
        }
        section{margin: 10px 0;}
        .info-box{margin: 10px auto;}
        .info-device-box .info-box{padding: 10px;}
        .num-box-mid div{margin: 10px 0;}
        .controlor-c > div{
            width: 270px;
            height: 270px;
        }
        .cc-btn{
            width: 90px;
            height: 90px;;
        }
        .big-num{font-size: 2em;}
        .controlor > div{margin: 40px 0;}
    }
    /* Radar Styles */
    #radar-section { text-align: center; background: #000; padding: 20px; border-radius: 8px; margin-top: 20px; }
    canvas { max-width: 100%; background: #000; border-radius: 50%; border: 1px solid #334155; }
    .status-dot { height: 10px; width: 10px; background: #ef4444; border-radius: 50%; display: inline-block; margin-right: 5px; }
    .connected { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
    .stats-overlay { font-family: monospace; color: #64748b; font-size: 12px; margin-bottom: 5px; }
    /* Autonome + Carte */
    #autonomous-section { background: rgba(30,41,59,0.6); border: 1px solid rgba(216,216,216,0.1); border-radius: 8px; padding: 16px; margin-top: 20px; }
    .toggle-row { display: flex; align-items: center; justify-content: center; gap: 12px; margin-bottom: 12px; flex-wrap: wrap; }
    .toggle-switch { position: relative; width: 52px; height: 28px; background: #475569; border-radius: 14px; cursor: pointer; }
    .toggle-switch.on { background: #2698EA; }
    .toggle-switch::after { content: ''; position: absolute; width: 22px; height: 22px; border-radius: 50%; background: #fff; top: 3px; left: 3px; transition: transform 0.2s; }
    .toggle-switch.on::after { transform: translateX(24px); }
    .raycasts-box { display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; margin: 10px 0; font-size: 11px; }
    .raycast-cell { min-width: 42px; padding: 4px 6px; background: rgba(0,0,0,0.3); border-radius: 4px; text-align: center; }
    .raycast-cell.low { background: rgba(239,68,68,0.4); color: #fca5a5; }
    .raycast-cell.mid { background: rgba(234,179,8,0.3); color: #fde047; }
    #map-section { text-align: center; margin-top: 12px; }
    #map-canvas { display: block; margin: 0 auto; background: #0f172a; border: 1px solid #334155; border-radius: 8px; max-width: 95%; touch-action: none; cursor: grab; }
    #map-canvas:active { cursor: grabbing; }
    .map-legend { font-size: 11px; color: #94a3b8; margin-top: 4px; }
    </style>
</head>
<body>
    <main>
        <section>
            <div>
                <h2 class="h2-tt" id="deviceInfo">Control Panel</h2>
            </div>
            <div class="for-move">
                <div class="for-move-a">
                    <div class="info-device-box">
                        <div class="info-box num-box-big">
                            <div >
                                <span class="big-num num-color" id="V">-1.01</span>
                                <span id="Vn">VOLTAGE</span>
                            </div>
                            <div>
                                <span class="big-num num-color" id="RSSI">-1.01</span>
                                <span id="RSSIn">RSSI</span>
                            </div>
                        </div>
                    </div>
                    <div class="info-device-box">
                        <div class="info-box num-box-mid">
                            <div>
                                <span class="num-color mid-num" id="r">-1.01</span>
                                <span id="rn">ROLL</span>
                            </div>
                            <div>
                                <span class="num-color mid-num" id="p">-1.01</span>
                                <span id="pn">PITCH</span>
                            </div>
                            <div>
                                <span class="num-color mid-num" id="y">-1.01</span>
                                <span id="yn">YAW</span>
                            </div>
                            <div>
                                <span class="num-color mid-num" id="mX">-1.01</span>
                                <span id="mXn">PAN</span>
                            </div>
                            <div>
                                <span class="num-color mid-num" id="mY">-1.01</span>
                                <span id="mYn">TILT</span>
                            </div>
                            <div>
                                <span class="num-color mid-num" id="mZ">-1.01</span>
                                <span id="mZn">SPD_R</span>
                            </div>
                        </div>
                    </div>
                    <div class="info-device-box">
                        <div class="info-box num-box-mid">
                            <div><span class="sma-num num-color" id="ax_ms2">--</span><span id="ax_ms2n">ax m/s²</span></div>
                            <div><span class="sma-num num-color" id="ay_ms2">--</span><span id="ay_ms2n">ay m/s²</span></div>
                            <div><span class="sma-num num-color" id="az_ms2">--</span><span id="az_ms2n">az m/s²</span></div>
                            <div><span class="sma-num num-color" id="gx">--</span><span id="gxn">gx °/s</span></div>
                            <div><span class="sma-num num-color" id="gy">--</span><span id="gyn">gy °/s</span></div>
                            <div><span class="sma-num num-color" id="gz">--</span><span id="gzn">gz °/s</span></div>
                        </div>
                        <div class="info-box num-box-sma" style="margin-top:8px;">
                            <div><span class="sma-num" id="ax">--</span><span>ax mg</span></div>
                            <div><span class="sma-num" id="ay">--</span><span>ay mg</span></div>
                            <div><span class="sma-num" id="az">--</span><span>az mg</span></div>
                            <div><span class="sma-num" id="mx">--</span><span>mx</span></div>
                            <div><span class="sma-num" id="my">--</span><span>my</span></div>
                            <div><span class="sma-num" id="mz">--</span><span>mz</span></div>
                        </div>
                    </div>
                    <div class="info-device-box">
                        <div class="info-box num-box-sma">
                            <div>
                                <span class="num-color sma-num" id="IP">192.168.10.67</span>
                                <span id="IPn">IP</span>
                            </div>
                            <div>
                                <span class="num-color sma-num" id="MAC">44:17:93:EE:F8:F8</span>
                                <span id="MACn">MAC</span>
                            </div>
                        </div>
                    </div>
                </div>
                <div class="for-move-b controlor">
                    <div class="controlor-c">
                        <div>
                            <div>
                                <label><button class="cc-btn" onmousedown="movtionButton(0.3,0.5);" ontouchstart="movtionButton(0.3,0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="23" height="23" viewBox="0 0 23 23"><g style="mix-blend-mode:passthrough"><path d="M0,2L0,18.1716C0,19.9534,2.15428,20.8457,3.41421,19.5858L19.5858,3.41421C20.8457,2.15428,19.9534,0,18.1716,0L2,0C0.895431,0,0,0.895431,0,2Z" fill="#D8D8D8" fill-opacity="0.20000000298023224"/></g></svg></button></label>
                                <label><button class="cc-btn" onmousedown="movtionButton(0.5,0.5);" ontouchstart="movtionButton(0.5,0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="26.87807685863504" height="15.435028109846826" viewBox="0 0 26.87807685863504 15.435028109846826"><g style="mix-blend-mode:passthrough" transform="matrix(0.9999999403953552,0,0,0.9999999403953552,0,0)"><path d="M12.0248,0.585787L0.589796,12.0208C-0.670133,13.2807,0.222199,15.435,2.00401,15.435L24.8741,15.435C26.6559,15.435,27.5482,13.2807,26.2883,12.0208L14.8533,0.585787C14.0722,-0.195262,12.8059,-0.195262,12.0248,0.585787Z" fill="#D8D8D8" fill-opacity="0.800000011920929"/></g></svg></button></label>
                                <label><button class="cc-btn" onmousedown="movtionButton(0.5,0.3);" ontouchstart="movtionButton(0.5,0.3);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="23" height="23" viewBox="0 0 23 23"><g style="mix-blend-mode:passthrough" transform="matrix(0,1,-1,0,23,-23)"><path d="M23,2L23,18.1716C23,19.9534,25.15428,20.8457,26.41421,19.5858L42.5858,3.41421C43.8457,2.15428,42.9534,0,41.1716,0L25,0C23.895431,0,23,0.895431,23,2Z" fill="#D8D8D8" fill-opacity="0.20000000298023224"/></g></svg></button></label>
                            </div>
                            <div>
                                <label><button class="cc-btn" onmousedown="movtionButton(-0.5,0.5);" ontouchstart="movtionButton(-0.5,0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="15.435028109846769" height="26.87807685863504" viewBox="0 0 15.435028109846769 26.87807685863504"><g style="mix-blend-mode:passthrough" transform="matrix(0.9999999403953552,0,0,0.9999999403953552,0,0)"><path d="M0.585787,14.8533L12.0208,26.2883C13.2807,27.5482,15.435,26.6559,15.435,24.8741L15.435,2.00401C15.435,0.222199,13.2807,-0.670133,12.0208,0.589795L0.585787,12.0248C-0.195262,12.8059,-0.195262,14.0722,0.585787,14.8533Z" fill="#D8D8D8" fill-opacity="0.800000011920929"/></g></svg></button></label>
                                <label><button class="cc-btn cc-middle" onmousedown="movtionButton(0,0);" ontouchstart="movtionButton(0,0);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);">STOP</button></label>
                                <label><button class="cc-btn" onmousedown="movtionButton(0.5,-0.5);" ontouchstart="movtionButton(0.5,-0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="15.435030017195288" height="26.87807685863504" viewBox="0 0 15.435030017195288 26.87807685863504"><g style="mix-blend-mode:passthrough" transform="matrix(0.9999999403953552,0,0,0.9999999403953552,0,0)"><path d="M14.8492,12.0248L3.41422,0.589796C2.15429,-0.670133,-9.53674e-7,0.222199,9.53674e-7,2.00401L9.53674e-7,24.8741C-9.53674e-7,26.6559,2.15429,27.5482,3.41421,26.2883L14.8492,14.8533C15.6303,14.0722,15.6303,12.8059,14.8492,12.0248Z" fill="#D8D8D8" fill-opacity="0.800000011920929"/></g></svg></button></label>
                            </div>
                            <div>
                                <label><button class="cc-btn" onmousedown="movtionButton(-0.3,-0.5);" ontouchstart="movtionButton(-0.3,-0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="23" height="23" viewBox="0 0 23 23"><g style="mix-blend-mode:passthrough" transform="matrix(0,-1,1,0,-23,23)"><path d="M0,25L0,41.1716C0,42.9534,2.15428,43.8457,3.41421,42.5858L19.5858,26.41421C20.8457,25.15428,19.9534,23,18.1716,23L2,23C0.895431,23,0,23.895431,0,25Z" fill="#D8D8D8" fill-opacity="0.20000000298023224"/></g></svg></button></label>
                                <label><button class="cc-btn" onmousedown="movtionButton(-0.5,-0.5);" ontouchstart="movtionButton(-0.5,-0.5);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="26.87807685863504" height="15.435030017195231" viewBox="0 0 26.87807685863504 15.435030017195231"><g style="mix-blend-mode:passthrough" transform="matrix(0.9999999403953552,0,0,0.9999999403953552,0,0)"><path d="M14.8533,14.8492L26.2883,3.41422C27.5482,2.15429,26.6559,-9.53674e-7,24.8741,9.53674e-7L2.00401,9.53674e-7C0.222199,-9.53674e-7,-0.670133,2.15429,0.589795,3.41421L12.0248,14.8492C12.8059,15.6303,14.0722,15.6303,14.8533,14.8492Z" fill="#D8D8D8" fill-opacity="0.800000011920929"/></g></svg></button></label>
                                <label><button class="cc-btn" onmousedown="movtionButton(-0.5,-0.3);" ontouchstart="movtionButton(-0.5,-0.3);" onmouseup="movtionButton(0,0);" ontouchend="movtionButton(0,0);"><svg fill="none" version="1.1" width="23" height="23" viewBox="0 0 23 23"><g style="mix-blend-mode:passthrough" transform="matrix(-1,0,0,-1,46,46)"><path d="M23,25L23,41.1716C23,42.9534,25.15428,43.8457,26.41421,42.5858L42.5858,26.41421C43.8457,25.15428,42.9534,23,41.1716,23L25,23C23.895431,23,23,23.895431,23,25Z" fill="#D8D8D8" fill-opacity="0.20000000298023224"/></g></svg></button></label>
                            </div>
                        </div>
                    </div>
                    <div class="control-speed">
                        <div>
                            <div id="device-gimbal-btn_A">
                                <label><button name="speedbtn" class="small-btn" onmousedown="gimbalCtrl(1);" ontouchstart="gimbalCtrl(1);" onmouseup="gimbalCtrl(0);" ontouchend="gimbalCtrl(0);">UP</button></label>
                            </div>
                        </div>
                        <br>
                        <div>
                            <div id="device-gimbal-btn_B">
                                <label><button name="speedbtn" class="small-btn" onmousedown="gimbalCtrl(3);" ontouchstart="gimbalCtrl(3);" onmouseup="gimbalCtrl(0);" ontouchend="gimbalCtrl(0);">LEFT</button></label>
                                <label><button name="speedbtn" class="small-btn" onmousedown="gimbalCtrl(5);" ontouchstart="gimbalCtrl(5);" onmouseup="gimbalCtrl(0);" ontouchend="gimbalCtrl(0);">FOEWARD</button></label>
                                <label><button name="speedbtn" class="small-btn" onmousedown="gimbalCtrl(4);" ontouchstart="gimbalCtrl(4);" onmouseup="gimbalCtrl(0);" ontouchend="gimbalCtrl(0);">RIGHT</button></label>
                            </div>
                        </div>
                        <br>
                        <div>
                            <div id="device-gimbal-btn_C">
                                <label><button name="speedbtn" class="small-btn" onmousedown="gimbalCtrl(2);" ontouchstart="gimbalCtrl(2);" onmouseup="gimbalCtrl(0);" ontouchend="gimbalCtrl(0);">DOWN</button></label>
                            </div>
                        </div>
                        <br>
                        <div>
                            <div id="device-gimbal-btn_D">
                                <label><button name="speedbtn" class="small-btn" onclick="gimbalSteady(1,read_Y);">STEADY START</button></label>
                                <label><button name="speedbtn" class="small-btn" onclick="gimbalSteady(0,read_Y);">STEADY END</button></label>
                            </div>
                        </div>
                        <br>
                        <br>
                        <div>
                            <div id="device-speed-btn">
                                <label><button name="speedbtn" class="small-btn" onclick="changeSpeed(0.3);">SLOW</button></label>
                                <label><button name="speedbtn" class="small-btn" onclick="changeSpeed(0.6);">MIDDLE</button></label>
                                <label><button name="speedbtn" class="small-btn" onclick="changeSpeed(1.0);">FAST</button></label>
                            </div>
                        </div>
                        <br>
                        <div>
                            <div id="device-led-btn">
                                <label><button name="speedbtn" class="small-btn" onclick="ledCtrl(1);">IO4</button></label>
                                <label><button name="speedbtn" class="small-btn" onclick="ledCtrl(2);">IO5</button></label>
                                <label><button name="speedbtn" class="small-btn" onclick="ledCtrl(0);">OFF</button></label>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </section>
        
        <!-- RADAR SECTION -->
        <section id="radar-section">
            <h2 class="h2-tt">Lidar Radar</h2>
            <div style="display: flex; align-items: center; justify-content: center; margin-bottom: 10px;">
                <div id="ws-dot" class="status-dot"></div>
                <span id="ws-text" style="font-size: 12px;">Déconnecté</span>
            </div>
            <div class="stats-overlay">
                PTS: <span id="pt-count">0</span> | FPS: <span id="fps-count">0</span>
            </div>
            <canvas id="radar" width="400" height="400"></canvas>
            <br>
            <input type="range" id="range-slider" min="1" max="12" value="6" step="0.5" style="width: 80%; margin-top: 10px;">
            <span id="range-val">6m</span>
        </section>

        <!-- AUTONOME + RAYCASTS + CARTE (calcul côté client = téléphone/PC, pas VM) -->
        <section id="autonomous-section">
            <h2 class="h2-tt">Mode autonome & Carte</h2>
            <div class="toggle-row">
                <span>Mode autonome</span>
                <div id="autonomous-toggle" class="toggle-switch" onclick="toggleAutonomous();" title="Raycasts devant, avance si libre, évite obstacles"></div>
                <span id="autonomous-label">OFF</span>
            </div>
            <div class="raycasts-box" id="raycasts-box">
                <span class="raycast-cell" data-angle="-45">-45°: --</span>
                <span class="raycast-cell" data-angle="-30">-30°: --</span>
                <span class="raycast-cell" data-angle="-15">-15°: --</span>
                <span class="raycast-cell" data-angle="0">0°: --</span>
                <span class="raycast-cell" data-angle="15">15°: --</span>
                <span class="raycast-cell" data-angle="30">30°: --</span>
                <span class="raycast-cell" data-angle="45">45°: --</span>
            </div>
            <p class="stats-overlay">Rayons = orientation uniquement (centrage + courbure murs). Obstacle frontal &lt; 400 mm → recul/tourne. Trajectoire = arc courbe. Carte = repère terrestre.</p>
            <div id="map-section">
                <canvas id="map-canvas" width="800" height="600"></canvas>
                <p class="map-legend">Glisser pour déplacer · Molette pour zoom · <button type="button" class="small-btn" onclick="clearMap();">Effacer la carte</button> <button type="button" class="small-btn" onclick="resetMapView();">Recentrer</button></p>
            </div>
        </section>

    </main>
<script>
    // --- RADAR LOGIC ---
    const canvas = document.getElementById('radar');
    const ctx = canvas.getContext('2d');
    const rangeSlider = document.getElementById('range-slider');
    const rangeVal = document.getElementById('range-val');
    
    let maxDist = 6000; 
    let points = [];
    let frames = 0;
    let lastTime = performance.now();
    let lastYaw = 0;
    let autonomousMode = false;
    let autonomousIntervalId = null;
    const OBSTACLE_MM = 400;
    const MAX_RAY_MM = 2500;
    const RAYCAST_ANGLES = [-45, -30, -15, 0, 15, 30, 45];
    const RAYCAST_ANGLES_FRONT = [-15, 0, 15];
    const ROBOT_BODY_MM = 80;
    const LIDAR_TO_REAR_MM = 25;
    const BOX_HALF_X_MM = 95;
    const BOX_HALF_Y_MM = 90;
    const PATH_PREDICT_STEP_M = 0.15;
    const PATH_PREDICT_N = 20;
    const PATH_PREDICT_TIME = 1.2;
    let lastTrajectoryArc = [];
    let lastTrajectoryTime = 0;
    const TRAJECTORY_PERSIST_MS = 600;
    let lastPlannedKappa = 0;
    const TRACK_CURVATURE_GAIN = 0.28;
    let pose = { x: 0, y: 0, yaw: 0 };
    let mapPoints = [];
    let mapOccupiedCells = new Set();
    let lastSentL = 0, lastSentR = 0, lastSentTime = 0;
    const POSE_SPEED_SCALE = 0.12;
    const MAP_MAX_POINTS = 800000;
    const MAP_CELL_M = 0.025;
    const MAP_SCALE_PX = 80;
    let mapPanX = 0, mapPanY = 0, mapZoom = 1;
    let mapDragging = false, mapLastX = 0, mapLastY = 0;
    let mapPinchStartDist = 0, mapPinchStartZoom = 1;
    let autonomousPhase = 'drive';
    let lastPlannedAngle = 0;
    let lastPlannedMagnitude = 0;

    rangeSlider.oninput = function() {
        maxDist = this.value * 1000;
        rangeVal.textContent = this.value + 'm';
    }

    function connectWs() {
        const ws = new WebSocket('ws://' + window.location.hostname + ':81');
        
        ws.onopen = () => {
            document.getElementById('ws-dot').classList.add('connected');
            document.getElementById('ws-text').textContent = "En ligne";
        };
        
        ws.onclose = () => {
            document.getElementById('ws-dot').classList.remove('connected');
            document.getElementById('ws-text').textContent = "Reconnexion...";
            setTimeout(connectWs, 2000);
        };

        ws.onmessage = (e) => {
            try {
                const data = JSON.parse(e.data);
                if (data.yaw != null) lastYaw = data.yaw;
                if(data.emergency) {
                    document.getElementById('ws-text').textContent = "⚠️ STOP OBSTACLE";
                    document.getElementById('ws-text').style.color = "red";
                } else {
                    document.getElementById('ws-text').textContent = "En ligne";
                    document.getElementById('ws-text').style.color = "#e2e8f0";
                }
                if(data.points) {
                    processPoints(data.points);
                    if (autonomousMode) updateMapFromPoints(data.points);
                }
            } catch(err) {}
        };
    }

    function isPointInsideRobotBody(a_deg, d_mm) {
        const ar = a_deg * Math.PI / 180;
        const x = d_mm * Math.cos(ar);
        const y = d_mm * Math.sin(ar);
        return (x >= -BOX_HALF_X_MM && x <= BOX_HALF_X_MM && y >= -BOX_HALF_Y_MM && y <= BOX_HALF_Y_MM);
    }

    function processPoints(raw) {
        let newPoints = [];
        for(let i=0; i<raw.length; i+=2) {
            newPoints.push({ a: raw[i], d: raw[i+1] });
        }
        points = newPoints;
        document.getElementById('pt-count').innerText = points.length;
    }

    function drawRadar() {
        frames++;
        const now = performance.now();
        if(now - lastTime >= 1000) {
            document.getElementById('fps-count').innerText = frames;
            frames = 0;
            lastTime = now;
        }

        const w = canvas.width;
        const h = canvas.height;
        const cx = w/2;
        const cy = h/2;
        const scale = (w/2) / maxDist; 

        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, w, h);

        ctx.strokeStyle = '#1e293b';
        ctx.lineWidth = 1;
        ctx.beginPath();
        const step = 1000; 
        for(let d=step; d<=maxDist; d+=step) {
            const r = d * scale;
            ctx.moveTo(cx + r, cy);
            ctx.arc(cx, cy, r, 0, Math.PI*2);
        }
        for(let a=0; a<360; a+=45) {
            const rad = a * Math.PI/180;
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + Math.cos(rad)*w, cy + Math.sin(rad)*w);
        }
        ctx.stroke();
        
        // Zone securite
        const safeR = 500 * scale;
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.5)'; 
        ctx.lineWidth = 2;
        ctx.arc(cx, cy, safeR, -Math.PI/2 - Math.PI/6, -Math.PI/2 + Math.PI/6);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(-Math.PI/2 - Math.PI/6)*safeR, cy + Math.sin(-Math.PI/2 - Math.PI/6)*safeR);
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(-Math.PI/2 + Math.PI/6)*safeR, cy + Math.sin(-Math.PI/2 + Math.PI/6)*safeR);
        ctx.stroke();

        // Rectangle zone à ignorer: +5 cm devant, +2 cm chaque côté (lidar au centre)
        const rx = cx - BOX_HALF_Y_MM * scale;
        const ry = cy - BOX_HALF_X_MM * scale;
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.7)';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(rx, ry, (2 * BOX_HALF_Y_MM) * scale, (2 * BOX_HALF_X_MM) * scale);

        // Raycasts autonome (limite MAX_RAY_MM pour rester centré en couloir)
        const ray = getRaycastsInFront();
        RAYCAST_ANGLES.forEach(function(a) {
            const rad = (a - 90) * Math.PI / 180;
            const rawDist = (ray[a] !== undefined && ray[a] < 99999) ? ray[a] : 99999;
            const dist = Math.min(rawDist, MAX_RAY_MM);
            const r = Math.min(dist, maxDist) * scale;
            const ex = cx + Math.cos(rad) * r;
            const ey = cy + Math.sin(rad) * r;
            const inZone = rawDist < MAX_RAY_MM && rawDist < OBSTACLE_MM;
            ctx.strokeStyle = inZone ? 'rgba(239, 68, 68, 0.8)' : 'rgba(34, 211, 238, 0.5)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(ex, ey);
            ctx.stroke();
            ctx.fillStyle = 'rgba(255,255,255,0.9)';
            ctx.font = '10px sans-serif';
            ctx.fillText(a + '°', ex + (Math.cos(rad) * 12), ey + (Math.sin(rad) * 12));
        });
        const maxRayR = MAX_RAY_MM * scale;
        ctx.strokeStyle = 'rgba(100, 116, 139, 0.4)';
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(cx, cy, maxRayR, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#22d3ee'; 
        for(let p of points) {
            if(p.d > maxDist) continue;
            if (isPointInsideRobotBody(p.a, p.d)) continue;
            const rad = (p.a - 90) * Math.PI/180;
            const r = p.d * scale;
            const x = cx + Math.cos(rad) * r;
            const y = cy + Math.sin(rad) * r;
            ctx.beginPath();
            ctx.arc(x, y, 2, 0, Math.PI*2);
            ctx.fill();
        }

        ctx.fillStyle = '#ef4444';
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, Math.PI*2);
        ctx.fill();

        // Trajectoire prévue PAR-DESSUS (dernier dessin = visible)
        const arc = getTrajectoryArcRover();
        if (arc.length > 0) {
            const trajScale = (w / 2) / 1.2;
            ctx.strokeStyle = 'rgba(34, 197, 94, 1)';
            ctx.lineWidth = 4;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            for (let i = 0; i < arc.length; i++) {
                const px = cx + arc[i].y * trajScale;
                const py = cy - arc[i].x * trajScale;
                ctx.lineTo(px, py);
            }
            ctx.stroke();
            ctx.fillStyle = 'rgba(34, 197, 94, 1)';
            for (let i = 0; i < arc.length; i++) {
                const px = cx + arc[i].y * trajScale;
                const py = cy - arc[i].x * trajScale;
                ctx.beginPath();
                ctx.arc(px, py, 6, 0, Math.PI*2);
                ctx.fill();
            }
        }

        requestAnimationFrame(drawRadar);
    }

    function getRaycastsInFront() {
        const sectorHalf = 10;
        const out = {};
        RAYCAST_ANGLES.forEach(a => { out[a] = 99999; });
        for (let i = 0; i < points.length; i += 1) {
            const p = points[i];
            if (isPointInsideRobotBody(p.a, p.d)) continue;
            const deg = p.a > 180 ? p.a - 360 : p.a;
            if (Math.abs(deg) > 55) continue;
            for (const a of RAYCAST_ANGLES) {
                if (Math.abs(deg - a) <= sectorHalf && p.d > 0 && p.d < out[a]) out[a] = p.d;
            }
        }
        return out;
    }

    function sendMotionCmd(l, r) {
        left_speed = l * speed_rate;
        right_speed = r * speed_rate;
        send_heartbeat = 1;
        lastSentL = l;
        lastSentR = r;
        lastSentTime = Date.now();
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "js?json=" + encodeURIComponent(JSON.stringify({ "T": 1, "L": left_speed, "R": right_speed })), true);
        xhr.send();
    }

    function autonomousLoop() {
        if (!autonomousMode) return;
        const ray = getRaycastsInFront();
        const eff = (a) => Math.min(ray[a] === 99999 ? 99999 : ray[a], MAX_RAY_MM);
        let minDistFront = 99999;
        RAYCAST_ANGLES_FRONT.forEach(a => {
            const d = ray[a] === 99999 ? 99999 : ray[a];
            if (d < MAX_RAY_MM && d < minDistFront) minDistFront = d;
        });
        const cells = document.querySelectorAll('.raycast-cell');
        RAYCAST_ANGLES.forEach((a, i) => {
            const d = ray[a];
            const mm = d === 99999 ? '>' + (MAX_RAY_MM/1000).toFixed(1) + 'm' : (d + '');
            cells[i].textContent = a + '°: ' + mm;
            cells[i].classList.remove('low', 'mid');
            if (d < MAX_RAY_MM && d < OBSTACLE_MM) cells[i].classList.add('low');
            else if (d < MAX_RAY_MM && d < 800) cells[i].classList.add('mid');
        });
        const frontBlocked = minDistFront < OBSTACLE_MM;
        if (frontBlocked) {
            let bestAngle = 0;
            RAYCAST_ANGLES.forEach(a => { if (eff(a) > eff(bestAngle)) bestAngle = a; });
            const turn = bestAngle < 0 ? -0.35 : 0.35;
            sendMotionCmd(turn, -turn);
            lastPlannedKappa = turn / TRACK_CURVATURE_GAIN;
            lastPlannedAngle = bestAngle;
            lastPlannedMagnitude = 0.35;
        } else {
            autonomousPhase = 'drive';
            const leftClear = (eff(-45) + eff(-30) + eff(-15)) / 3;
            const rightClear = (eff(15) + eff(30) + eff(45)) / 3;
            const leftSlope = (eff(-15) - eff(-45)) / 30;
            const rightSlope = (eff(15) - eff(45)) / 30;
            const kappaCenter = (leftClear - rightClear) * 0.00035;
            const kappaSlope = (leftSlope - rightSlope) * 0.0008;
            let kappa = kappaCenter + kappaSlope;
            kappa = Math.max(-0.9, Math.min(0.9, kappa));
            lastPlannedKappa = kappa;
            const base = 0.4;
            const bl = Math.max(0.12, Math.min(0.5, base - kappa * TRACK_CURVATURE_GAIN));
            const br = Math.max(0.12, Math.min(0.5, base + kappa * TRACK_CURVATURE_GAIN));
            sendMotionCmd(bl, br);
            lastPlannedAngle = Math.atan2(br - bl, br + bl) * 180 / Math.PI;
            lastPlannedMagnitude = (bl + br) / 2;
        }
    }

    function updateMapFromPoints(rawPoints) {
        const now = Date.now();
        const yawRad = lastYaw * Math.PI / 180;
        if (lastSentTime > 0) {
            const dt = (now - lastSentTime) / 1000;
            const v = (lastSentL + lastSentR) / 2 * speed_rate * POSE_SPEED_SCALE;
            const yawPrev = pose.yaw;
            pose.x += v * dt * Math.cos(yawPrev);
            pose.y += v * dt * Math.sin(yawPrev);
        }
        pose.yaw = yawRad;
        for (let i = 0; i < rawPoints.length; i += 2) {
            const a = rawPoints[i], d = rawPoints[i + 1];
            if (d < 50 || d > 8000) continue;
            if (isPointInsideRobotBody(a, d)) continue;
            const ar = a * Math.PI / 180;
            const rx = (d / 1000) * Math.cos(ar);
            const ry = (d / 1000) * Math.sin(ar);
            const wx = pose.x + Math.cos(pose.yaw) * rx - Math.sin(pose.yaw) * ry;
            const wy = pose.y + Math.sin(pose.yaw) * rx + Math.cos(pose.yaw) * ry;
            const ci = Math.floor(wx / MAP_CELL_M);
            const cj = Math.floor(wy / MAP_CELL_M);
            const cellKey = ci + ',' + cj;
            if (mapOccupiedCells.has(cellKey)) continue;
            mapOccupiedCells.add(cellKey);
            mapPoints.push({ x: wx, y: wy });
            if (mapPoints.length > MAP_MAX_POINTS) mapPoints.splice(0, 5000);
        }
    }

    function getTrajectoryArcRover() {
        const pts = [];
        let v, k;
        if (autonomousMode) {
            v = lastPlannedMagnitude * speed_rate * POSE_SPEED_SCALE;
            k = lastPlannedKappa;
        } else {
            v = (lastSentL + lastSentR) / 2 * speed_rate * POSE_SPEED_SCALE;
            k = (Math.abs(lastSentL + lastSentR) < 0.01) ? 0 : (lastSentR - lastSentL) / (2 * TRACK_CURVATURE_GAIN);
        }
        if (v < 0.02 && Math.abs(k) < 0.05) {
            if (lastTrajectoryArc.length > 0 && (Date.now() - lastTrajectoryTime) < TRAJECTORY_PERSIST_MS)
                return lastTrajectoryArc;
            return [];
        }
        if (v < 0.02) v = 0.06;
        const n = PATH_PREDICT_N;
        const step = (v * PATH_PREDICT_TIME) / n;
        for (let i = 1; i <= n; i++) {
            const s = step * i;
            let rx, ry;
            if (Math.abs(k) < 0.02) {
                rx = s;
                ry = 0;
            } else {
                const th = k * s;
                rx = Math.sin(th) / k;
                ry = (1 - Math.cos(th)) / k;
            }
            pts.push({ x: rx, y: ry });
        }
        lastTrajectoryArc = pts;
        lastTrajectoryTime = Date.now();
        return pts;
    }

    function getPathPointsWorld() {
        const arc = getTrajectoryArcRover();
        const out = [];
        const c = Math.cos(pose.yaw);
        const s = Math.sin(pose.yaw);
        for (let i = 0; i < arc.length; i++) {
            const rx = arc[i].x, ry = arc[i].y;
            out.push({
                x: pose.x + c * rx - s * ry,
                y: pose.y + s * rx + c * ry
            });
        }
        return out;
    }

    function drawMap() {
        const mc = document.getElementById('map-canvas');
        if (!mc) return;
        const mctx = mc.getContext('2d');
        const cw = mc.width, ch = mc.height;
        const cx = cw / 2, cy = ch / 2;
        const sc = MAP_SCALE_PX * mapZoom;
        const ox = cx + mapPanX;
        const oy = cy + mapPanY;
        mctx.fillStyle = '#0f172a';
        mctx.fillRect(0, 0, cw, ch);
        mctx.fillStyle = 'rgba(34, 211, 238, 0.45)';
        for (let i = 0; i < mapPoints.length; i++) {
            const px = ox + mapPoints[i].x * sc;
            const py = oy - mapPoints[i].y * sc;
            if (px >= -2 && px < cw + 2 && py >= -2 && py < ch + 2) {
                mctx.fillRect(Math.round(px), Math.round(py), Math.max(1, Math.round(2 * mapZoom)), Math.max(1, Math.round(2 * mapZoom)));
            }
        }
        const pathPts = getTrajectoryArcRover().length > 0 ? getPathPointsWorld() : [];
        if (pathPts.length > 0) {
            mctx.strokeStyle = 'rgba(34, 197, 94, 0.95)';
            mctx.lineWidth = Math.max(1, 2 * mapZoom);
            mctx.beginPath();
            mctx.moveTo(ox + pose.x * sc, oy - pose.y * sc);
            for (let i = 0; i < pathPts.length; i++) {
                mctx.lineTo(ox + pathPts[i].x * sc, oy - pathPts[i].y * sc);
            }
            mctx.stroke();
            mctx.fillStyle = 'rgba(34, 197, 94, 0.95)';
            for (let i = 0; i < pathPts.length; i++) {
                const px = ox + pathPts[i].x * sc;
                const py = oy - pathPts[i].y * sc;
                if (px >= -10 && px < cw + 10 && py >= -10 && py < ch + 10) {
                    mctx.beginPath();
                    mctx.arc(px, py, Math.max(2, 4 * mapZoom), 0, Math.PI*2);
                    mctx.fill();
                }
            }
        }
        mctx.save();
        mctx.translate(ox + pose.x * sc, oy - pose.y * sc);
        mctx.rotate(-pose.yaw);
        mctx.fillStyle = '#2698EA';
        mctx.strokeStyle = '#1e40af';
        mctx.lineWidth = 1;
        mctx.beginPath();
        mctx.moveTo(12, 0);
        mctx.lineTo(-8, -6);
        mctx.lineTo(-8, 6);
        mctx.closePath();
        mctx.fill();
        mctx.stroke();
        mctx.restore();
        mctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
        mctx.font = '11px sans-serif';
        mctx.fillText('Zoom: ' + mapZoom.toFixed(1) + ' · ' + mapPoints.length + ' pts', 8, ch - 6);
        requestAnimationFrame(drawMap);
    }

    function resetMapView() {
        mapPanX = 0;
        mapPanY = 0;
        mapZoom = 1;
    }

    function setupMapNav() {
        const mc = document.getElementById('map-canvas');
        if (!mc) return;
        mc.addEventListener('wheel', function(e) {
            e.preventDefault();
            const d = e.deltaY > 0 ? -0.15 : 0.15;
            mapZoom = Math.max(0.2, Math.min(5, mapZoom * (1 + d)));
        }, { passive: false });
        mc.addEventListener('mousedown', function(e) {
            if (e.button === 0) { mapDragging = true; mapLastX = e.clientX; mapLastY = e.clientY; }
        });
        mc.addEventListener('mousemove', function(e) {
            if (mapDragging) {
                mapPanX += e.clientX - mapLastX;
                mapPanY += e.clientY - mapLastY;
                mapLastX = e.clientX;
                mapLastY = e.clientY;
            }
        });
        mc.addEventListener('mouseup', function(e) { if (e.button === 0) mapDragging = false; });
        mc.addEventListener('mouseleave', function() { mapDragging = false; });
        function dist2(a, b) {
            return (a.clientX - b.clientX) ** 2 + (a.clientY - b.clientY) ** 2;
        }
        mc.addEventListener('touchstart', function(e) {
            if (e.touches.length === 1) {
                mapDragging = true;
                mapLastX = e.touches[0].clientX;
                mapLastY = e.touches[0].clientY;
            } else if (e.touches.length === 2) {
                mapDragging = false;
                mapPinchStartDist = Math.sqrt(dist2(e.touches[0], e.touches[1]));
                mapPinchStartZoom = mapZoom;
            }
        }, { passive: true });
        mc.addEventListener('touchmove', function(e) {
            if (e.touches.length === 2) {
                const d = Math.sqrt(dist2(e.touches[0], e.touches[1]));
                if (mapPinchStartDist > 0) {
                    mapZoom = Math.max(0.2, Math.min(5, mapPinchStartZoom * (d / mapPinchStartDist)));
                }
            } else if (e.touches.length === 1 && mapDragging) {
                mapPanX += e.touches[0].clientX - mapLastX;
                mapPanY += e.touches[0].clientY - mapLastY;
                mapLastX = e.touches[0].clientX;
                mapLastY = e.touches[0].clientY;
            }
        }, { passive: true });
        mc.addEventListener('touchend', function(e) {
            if (e.touches.length === 0) { mapDragging = false; mapPinchStartDist = 0; }
            else if (e.touches.length === 1) {
                mapDragging = true;
                mapLastX = e.touches[0].clientX;
                mapLastY = e.touches[0].clientY;
            } else if (e.touches.length === 2) {
                mapPinchStartDist = Math.sqrt(dist2(e.touches[0], e.touches[1]));
                mapPinchStartZoom = mapZoom;
            }
        }, { passive: true });
    }

    function clearMap() {
        mapPoints = [];
        mapOccupiedCells = new Set();
        pose = { x: 0, y: 0, yaw: 0 };
        lastSentTime = 0;
        autonomousPhase = 'drive';
        resetMapView();
    }

    function toggleAutonomous() {
        autonomousMode = !autonomousMode;
        const el = document.getElementById('autonomous-toggle');
        const label = document.getElementById('autonomous-label');
        if (autonomousMode) {
            el.classList.add('on');
            label.textContent = 'ON';
            sendMotionCmd(0, 0);
            autonomousIntervalId = setInterval(autonomousLoop, 120);
        } else {
            el.classList.remove('on');
            label.textContent = 'OFF';
            if (autonomousIntervalId) clearInterval(autonomousIntervalId);
            autonomousPhase = 'drive';
            lastPlannedMagnitude = 0;
            sendMotionCmd(0, 0);
        }
    }

    connectWs();
    drawRadar();
    drawMap();
    setupMapNav();

    // --- END RADAR LOGIC ---

    var cmdA;
    var cmdB;
    var cmdC;

    var forwardButton;   // 1
    var backwardButton;  // 2
    var fbNewer;

    var leftButton;      // 1
    var rightButton;     // 2
    var lrNewer;

    var last_forwardButton;   // 1
    var last_backwardButton;  // 2
    var last_fbNewer;

    var last_leftButton;      // 1
    var last_rightButton;     // 2
    var last_lrNewer;

    var speed_rate  = 1; // 1:fast 0.6:middle 0.3:slow
    var left_speed  = 0;
    var right_speed = 0;
    var send_heartbeat = 0;

    var io4_status = 0;
    var io5_status = 0;

    var gimbal_T = 135;
    var gimbal_X = 0;
    var gimbal_Y = 0;
    var read_X = 0;
    var read_Y = 0;

    var steady_status = 0;
    var steady_bias   = 0;

    getDevInfo();
    ledCtrl(0);
    setInterval(function() {
        getDevInfo();
    }, 2510);

    setInterval(function() {
        heartBeat();
    }, 1500);

    setInterval(function() {
        infoUpdate();
    }, 1000);

    function infoUpdate() {
        var jsonCmd = {
            "T": 130
        }
        var jsonString = JSON.stringify(jsonCmd);
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                var jsonResponse = JSON.parse(this.responseText);
                document.getElementById("V").innerHTML = jsonResponse.v.toFixed(2);
                if (jsonResponse.V<11.06) {
                    document.getElementById("V").classList.remove("num-color");
                    document.getElementById("V").classList.add("num-color-red");
                }else{
                    document.getElementById("V").classList.remove("num-color-red");
                    document.getElementById("V").classList.add("num-color");
                }

                document.getElementById("r").innerHTML = jsonResponse.r?.toFixed(2);
                document.getElementById("p").innerHTML = jsonResponse.p?.toFixed(2);
                document.getElementById("y").innerHTML = jsonResponse.y?.toFixed(2);
                document.getElementById("mZ").innerHTML = speed_rate;

                if (jsonResponse.ax_ms2 != null) document.getElementById("ax_ms2").innerHTML = jsonResponse.ax_ms2.toFixed(3);
                if (jsonResponse.ay_ms2 != null) document.getElementById("ay_ms2").innerHTML = jsonResponse.ay_ms2.toFixed(3);
                if (jsonResponse.az_ms2 != null) document.getElementById("az_ms2").innerHTML = jsonResponse.az_ms2.toFixed(3);
                if (jsonResponse.gx != null) document.getElementById("gx").innerHTML = jsonResponse.gx.toFixed(2);
                if (jsonResponse.gy != null) document.getElementById("gy").innerHTML = jsonResponse.gy.toFixed(2);
                if (jsonResponse.gz != null) document.getElementById("gz").innerHTML = jsonResponse.gz.toFixed(2);
                if (jsonResponse.ax != null) document.getElementById("ax").innerHTML = jsonResponse.ax.toFixed(0);
                if (jsonResponse.ay != null) document.getElementById("ay").innerHTML = jsonResponse.ay.toFixed(0);
                if (jsonResponse.az != null) document.getElementById("az").innerHTML = jsonResponse.az.toFixed(0);
                if (jsonResponse.mx != null) document.getElementById("mx").innerHTML = jsonResponse.mx;
                if (jsonResponse.my != null) document.getElementById("my").innerHTML = jsonResponse.my;
                if (jsonResponse.mz != null) document.getElementById("mz").innerHTML = jsonResponse.mz;

                if (jsonResponse.hasOwnProperty('pan')) {
                    document.getElementById("mX").innerHTML = jsonResponse.pan?.toFixed(2);
                    document.getElementById("mY").innerHTML = jsonResponse.tilt?.toFixed(2);

                    read_X = jsonResponse.pan;
                    read_Y = jsonResponse.tilt;
                } else {
                    document.getElementById("mX").innerHTML = "—";
                    document.getElementById("mY").innerHTML = "—";

                    read_X = 0;
                    read_Y = 0;
                }
            }
        };
        xhttp.open("GET", "js?json=" + jsonString, true);
        xhttp.send();
    }
    function getDevInfo() {
        var jsonCmd = {
            "T": 405
        }
        var jsonString = JSON.stringify(jsonCmd);
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
            if (this.readyState == 4 && this.status == 200) {
                var jsonResponse = JSON.parse(this.responseText);

                document.getElementById("IP").innerHTML = jsonResponse.ip;
                document.getElementById("MAC").innerHTML = jsonResponse.mac;
                document.getElementById("RSSI").innerHTML = jsonResponse.rssi;
            }
        };
        xhttp.open("GET", "js?json=" + jsonString, true);
        xhttp.send();
    }
    function changeSpeed(inputSpd) {
        speed_rate = inputSpd;
    }
    function heartBeat() {
        if (send_heartbeat == 1) {
            lastSentL = left_speed / speed_rate;
            lastSentR = right_speed / speed_rate;
            lastSentTime = Date.now();
            var jsonCmd = {
                "T":1,
                "L":left_speed,
                "R":right_speed
            }
            var jsonString = JSON.stringify(jsonCmd);
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "js?json=" + jsonString, true);
            xhr.send();
        }
    }
    function movtionButton(spdL, spdR){
        left_speed  = spdL*speed_rate;
        right_speed = spdR*speed_rate;
        send_heartbeat = 1;
        lastSentL = spdL;
        lastSentR = spdR;
        lastSentTime = Date.now();
        var jsonCmd = {
            "T":1,
            "L":left_speed,
            "R":right_speed
        }
        var jsonString = JSON.stringify(jsonCmd);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "js?json=" + jsonString, true);
        xhr.send();
    }
    function ledCtrl(inputCmd){
        if (inputCmd == 0) {
            io4_status = 0;
            io5_status = 0;
        }
        else if (inputCmd == 1) {
            if (io4_status == 0) {
                io4_status = 255;
            }
            else {
                io4_status = 0;
            }
        }
        else if (inputCmd == 2) {
            if (io5_status == 0) {
                io5_status = 255;
            }
            else {
                io5_status = 0;
            }
        }
        var jsonCmd = {
            "T":132,
            "IO4":io4_status,
            "IO5":io5_status
        }
        var jsonString = JSON.stringify(jsonCmd);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "js?json=" + jsonString, true);
        xhr.send();
    }
    function gimbalSteady(inputS,inputY){
        steady_status = inputS;
        steady_bias = inputY;
        var jsonCmd = {
            "T":137,
            "s":steady_status,
            "y":steady_bias
        }
        var jsonString = JSON.stringify(jsonCmd);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "js?json=" + jsonString, true);
        xhr.send();
    }
    function gimbalCtrl(inputCmd){
        if (inputCmd == 0) {
            gimbal_T = 135;
        }else if (inputCmd == 1) {
            gimbal_T = 134;
            gimbal_X = read_X;
            gimbal_Y = 90;
            if (steady_status == 1) {
                steady_bias = steady_bias + 5;
                if (steady_bias > 90) {
                    steady_bias = 90;
                }
            }
        }else if (inputCmd == 2) {
            gimbal_T = 134;
            gimbal_X = read_X;
            gimbal_Y = -45;
            if (steady_status == 1) {
                steady_bias = steady_bias - 5;
                if (steady_bias < -45) {
                    steady_bias = 45;
                }
            }
        }else if (inputCmd == 3) {
            gimbal_T = 134;
            gimbal_X = -180;
            gimbal_Y = read_Y;
        }else if (inputCmd == 4) {
            gimbal_T = 134;
            gimbal_X = 180;
            gimbal_Y = read_Y;
        }else if (inputCmd == 5) {
            gimbal_T = 134;
            gimbal_X = 0;
            gimbal_Y = 0;
            if (steady_status == 1) {
                steady_bias = 0;
            }
        }

        if (steady_status == 0) {
            var jsonCmd = {
                "T":gimbal_T,
                "X":gimbal_X,
                "Y":gimbal_Y,
                "SX":600,
                "SY":600
            }
            var jsonString = JSON.stringify(jsonCmd);
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "js?json=" + jsonString, true);
            xhr.send();
        }else if (steady_status == 1) {
            gimbalSteady(1,steady_bias);
        }
    }

    function cmdProcess(){
        if (forwardButton == 0 && backwardButton == 0 && leftButton == 0 && rightButton == 0) {
            movtionButton(0, 0);
        }
        else if (forwardButton == 1 && backwardButton == 0 && leftButton == 0 && rightButton == 0){
            movtionButton(0.5, 0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 1 && leftButton == 0 && rightButton == 0){
            movtionButton(0.5, 0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 2 && leftButton == 0 && rightButton == 0){
            movtionButton(-0.5, -0.5);
        }else if (forwardButton == 0 && backwardButton == 1 && leftButton == 0 && rightButton == 0){
            movtionButton(-0.5, -0.5);
        }
        else if (forwardButton == 0 && backwardButton == 0 && leftButton == 1 && rightButton == 0){
            movtionButton(-0.5, 0.5);
        }else if (forwardButton == 0 && backwardButton == 0 && leftButton == 1 && rightButton == 1 && lrNewer == 1){
            movtionButton(-0.5, 0.5);
        }else if (forwardButton == 0 && backwardButton == 0 && leftButton == 0 && rightButton == 1){
            movtionButton(0.5, -0.5);
        }else if (forwardButton == 0 && backwardButton == 0 && leftButton == 1 && rightButton == 1 && lrNewer == 2){
            movtionButton(0.5, -0.5);
        }
        else if (forwardButton == 1 && backwardButton == 0 && leftButton == 1 && rightButton == 0){
            movtionButton(0.3, 0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 1 && leftButton == 1 && rightButton == 0){
            movtionButton(0.3, 0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 1 && leftButton == 1 && rightButton == 1 && lrNewer == 1){
            movtionButton(0.3, 0.5);
        }else if (forwardButton == 1 && backwardButton == 0 && leftButton == 1 && rightButton == 1 && lrNewer == 1){
            movtionButton(0.3, 0.5);
        }
        else if (forwardButton == 1 && backwardButton == 0 && leftButton == 0 && rightButton == 1){
            movtionButton(0.5, 0.3);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 1 && leftButton == 0 && rightButton == 1){
            movtionButton(0.5, 0.3);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 1 && leftButton == 1 && rightButton == 1 && lrNewer == 2){
            movtionButton(0.5, 0.3);
        }else if (forwardButton == 1 && backwardButton == 0 && leftButton == 1 && rightButton == 1 && lrNewer == 2){
            movtionButton(0.5, 0.3);
        }
        else if (forwardButton == 0 && backwardButton == 1 && leftButton == 1 && rightButton == 0){
            movtionButton(-0.3, -0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 2 && leftButton == 1 && rightButton == 0){
            movtionButton(-0.3, -0.5);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 2 && leftButton == 1 && rightButton == 1 && lrNewer == 1){
            movtionButton(-0.3, -0.5);
        }else if (forwardButton == 0 && backwardButton == 1 && leftButton == 1 && rightButton == 1 && lrNewer == 1){
            movtionButton(-0.3, -0.5);
        }
        else if (forwardButton == 0 && backwardButton == 1 && leftButton == 0 && rightButton == 1){
            movtionButton(-0.5, -0.3);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 2 && leftButton == 0 && rightButton == 1){
            movtionButton(-0.5, -0.3);
        }else if (forwardButton == 1 && backwardButton == 1 && fbNewer == 2 && leftButton == 1 && rightButton == 1 && lrNewer == 2){
            movtionButton(-0.5, -0.3);
        }else if (forwardButton == 0 && backwardButton == 1 && leftButton == 1 && rightButton == 1 && lrNewer == 2){
            movtionButton(-0.5, -0.3);
        }
    }

    document.onkeydown = function (event) {
        var e = event || window.event || arguments.callee.caller.arguments[0];
        if (e && e.keyCode == 65) {
            // alert ("A down");
            leftButton = 1;
            lrNewer = 1;
        }else if (e && e.keyCode == 87) {
            // alert ("W down");
            forwardButton = 1;
            fbNewer = 1;
        }else if (e && e.keyCode == 83) {
            // alert ("S down");
            backwardButton = 1;
            fbNewer = 2;
        }        else if (e && e.keyCode == 68) {
            // alert ("D down");
            rightButton = 1;
            lrNewer = 2;
        }
        else if (e && e.keyCode == 37) {
            // alert ("left down");
            leftButton = 1;
            lrNewer = 1;
        }else if (e && e.keyCode == 38) {
            // alert ("up down");
            forwardButton = 1;
            fbNewer = 1;
        }else if (e && e.keyCode == 40) {
            // alert ("down down");
            backwardButton = 1;
            fbNewer = 2;
        }else if (e && e.keyCode == 39) {
            // alert ("right down");
            rightButton = 1;
            lrNewer = 2;
        }

        if(forwardButton != last_forwardButton || backwardButton != last_backwardButton || fbNewer != last_fbNewer || leftButton != last_leftButton || last_rightButton != rightButton || lrNewer != last_fbNewer) {
            cmdProcess();
            last_forwardButton = forwardButton;
            last_backwardButton = backwardButton;
            last_fbNewer = fbNewer;

            last_leftButton = leftButton;
            last_rightButton = rightButton;
            last_lrNewer = lrNewer;
        }
    }

    document.onkeyup = function (event) {
        var e = event || window.event || arguments.callee.caller.arguments[0];
        if (e && e.keyCode == 65) {
            // alert ("A up");
            leftButton = 0;
        }else if (e && e.keyCode == 87) {
            // alert ("W up");
            forwardButton = 0;
        }else if (e && e.keyCode == 83) {
            // alert ("S up");
            backwardButton = 0;
        }else if (e && e.keyCode == 68) {
            // alert ("D up");
            rightButton = 0;
        }
        else if (e && e.keyCode == 37) {
            // alert ("left up");
            leftButton = 0;
        }else if (e && e.keyCode == 38) {
            // alert ("up up");
            forwardButton = 0;
        }else if (e && e.keyCode == 40) {
            // alert ("down up");
            backwardButton = 0;
        }else if (e && e.keyCode == 39) {
            // alert ("right up");
            rightButton = 0;
        }

        cmdProcess();

        last_forwardButton = forwardButton;
        last_backwardButton = backwardButton;
        last_fbNewer = fbNewer;

        last_leftButton = leftButton;
        last_rightButton = rightButton;
        last_lrNewer = lrNewer;
    }
</script>
</body>
</html>
)rawliteral";