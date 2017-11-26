// --------------------------------------------------------------------------------
// ASCOM Camera driver low-level interaction library for cam86 v.0.9.1L
// Edit Log:
// Date Who Vers Description
// ----------- --- ----- ---------------------------------------------------------
// 28-aug-2016 VSS 0.1 Initial release (code obtained from grim)
//
// 1-feb-2017 Luka Pravica 0.2L
//     - Update code to match the newer cam86_view
//     - Add function to set camera reading time
//     - if camera is not idle cameraGetCoolerPower, cameraGetCoolerOn and
//       cameraGetTemp return cashed values to prevent image corruption while
//       downloading images
// 3-feb-2017 Luka Pravica 0.3L
//     - Add function to set if TEC cooling should be on or off during frame reading
//     - Add function to read firmware version
//     - Add function to read version of this DLL
// 19-feb-2017 Luka Pravica 0.4L
//     - Add bias-before-exposure option
// 2-mar-2017 Luka Pravica 0.5L
//     - Add humidity functions by Gilmanov Rim
//     - Add controls for maximum and starting TEC power
// 8-mar-2017 Luka Pravica 0.6L
//     - Fix bug where timer stops working if time is greater than 999s
//       by using multiple restarts of the timer for every 900s
// 11-mar-2017 Luka Pravica 0.7L
//     - Fix bug in long exposures (over 900s)
//     - Add code to debug to file (must be disabled in production version)
// 17-mar-2017 Luka Pravica 0.8L
//     - Add code to set the KP proportional gain
// 20-mar-2017 Luka Pravica 0.9L
//     - Add code to read Cooler min, cooler max and Pk
// 21-mar-2017 Luka Pravica 0.9.1L
//     - Fix bugs in Pk reading
// 27-Sep-2017 Luka Pravica 0.9.2L
//     - Add caching of setCCDtemperature Get/Set and CoolerOn Get/Set during frame reads to prevent the white line bug 
// --------------------------------------------------------------------------------

{  Copyright � 2017 Gilmanov Rim, Vakulenko Sergiy and Luka Pravica

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
}

library cam86ll;

uses
  Classes,
  SysUtils,
  MyD2XX,
  MMSystem,
  Windows,
  SyncObjs,
  ExtCtrls;

{$R *.res}

const

    debugOn = false;
    //debugFileName: string = 'cam86_log.txt';
    debugFileName: string = 'C:\Users\user\Desktop\cam86_log.txt';

    softwareLLDriverVersion = 92;

    //ширина изображения
    CameraWidth  = 3000;
    //высота изображения
    CameraHeight = 2000; 
    //первоначальное значение на выводах порта BDBUS
    portfirst = $11;
    xccd = 1500;
    yccd = 1000;
    //bitbang speed
    spusb = 20000;

    //camera state consts
    cameraIdle = 0;
    cameraWaiting = 1;
    cameraExposing = 2;
    cameraReading = 3;
    cameraDownload = 4;
    cameraError = 5;

    TemperatureOffset = 1280;
    MinErrTemp = -120.0;
    MaxErrTemp = 120.0;
    TRUE_INV_PROT = $aa55;
    FALSE_INV_PROT = $55aa;
    HIGH_MASK_PROT = $aa00;

    //driver image type
    type camera_image_type = array [0..CameraHeight*CameraWidth-1] of integer;

    //Class for reading thread
    posl = class(TThread)
    private
    //Private declarations
    protected
    procedure Execute; override;
    end;

//GLobal variables}
var
    debugTextFile: TextFile;
    debugTextFileIsOpen: boolean = False;

    //переменная-флаг, отображает состояние соединения с камерой
    isConnected : boolean = false;
    //указатель текущего адреса в выходном буфере FT2232HL
    adress : integer;
    //биннинг,
    mBin : integer;
    //переменная-флаг, отображает готовность к считыванию кадра
    imageReady : boolean = false;
    //переменная-состояние камеры  0 - ready 1 - longexp 2 - read
    cameraState : integer = 0;
    //таймер экспозиции
    ExposureTimer : integer;
    //переменная для второго потока (чтение изображения)
    co : posl;
    //буферный массив-изображение для операций
    bufim : camera_image_type;
    //начало чтения и количество по строкам
    mYn, mdeltY : integer;
    kolbyte : integer;
    eexp : integer;

    siin : array[0..3] of byte;
    siout : word;

    //error Flag
    errorReadFlag : boolean;
    errorWriteFlag : boolean;

    //cached values
    sensorTempCache : Double = 0;
    targetTempCache : Double = 0;
    targetTempDirty : Boolean = false;
    coolerOnCache : WordBool = false;
    coolerOnDirty : Boolean = false;
    coolerPowerCache : Double = 0;
    firmwareVersionCache : byte = 0;
    tempDHTCache : Double = -128.0;
    humidityDHTCache : Double = -1;
    CoolingStartingPowerPercentageCache : integer = -1;
    CoolingMaximumPowerPercentageCache : integer = 101;
    KpCache : Double = 0.0;


    // timer counter
    // timer can only count <1000s
    // for longer exposures repeat the timer run as needed
    exposure_time_left : integer;
    eposure_time_rollover : integer = 900000; // 900 seconds (999 seconds is max)
    exposure_time_loop_counter : integer;

    // used when 0s exposure image is taken to clear the sensor before real exposure
    sensorClear : boolean;

// needed function declarations
function cameraSetTemp(temp : double): WordBool; stdcall; export; forward;
function cameraCoolingOn (): WordBool; stdcall; export; forward;
function cameraCoolingOff (): WordBool; stdcall; export; forward;

// -------------------------------------------------------------------------------
// Infrastructure part
// -------------------------------------------------------------------------------

// Небольшое пояснение работы с FT2232LH.
// Всегда используется такой прием:
//  1. Вначале заполняется буфер и исходными байтами (необходимой последовательности импульсов на выводах порта BDBUS).
// При этом инкрементируется указатель adress.
//  2. Далее весь этот массив передается на выход командой: n:=Write_USB_Device_Buffer(FT_CAM8B,adress);
// Замечательная микросхема FT2232HL честно без задержек все это передает на свой порт // BDBUS. Передача 1 байта при этом занимает 65 нс.
// Время отработки следующей команды n:=Write_USB_Device_Buffer(FT_CAM8B,adress) зависит // от загруженности операционки и не контролируется
// нами. Поэтому критическую последовательности импульсов нужно заполнять всю, а не передавать по очереди.
// Благо програмный буфер драйвера это позволяет (в этой программе до 24 Мбайт!) Для этого нужно изменить текст D2XX.pas, я назвал его MyD2XX.pas

procedure debugToFile(line: string);
begin
    if debugOn then
    begin
        if not debugTextFileIsOpen then
        begin
            AssignFile(debugTextFile, debugFileName);
            if FileExists(debugFileName) = False then
                Rewrite(debugTextFile)
            else begin
                Reset(debugTextFile);
                Append(debugTextFile);
            end;
            debugTextFileIsOpen := True;
        end;
        WriteLn(debugTextFile, DateToStr(Now) + TimeToStr(Now) + ': ' + line);
        Flush(debugTextFile);
        CloseFile(debugTextFile);
        debugTextFileIsOpen := False;
    end;
end;

function Qbuf():integer;
begin
    Get_USB_Device_QueueStatus(FT_HANDLEA);
    result:=FT_Q_Bytes;
end;

procedure sspi;
var i,j:integer;
    b:byte;
    n:word;
begin
    n:=100;
    FillChar(FT_Out_Buffer,n,portfirst);
    for j:=0 to 2 do
    begin
        b:=siin[j];
        for i:= 0 to 7 do
        begin
            inc(FT_Out_Buffer[2*i+1+16*j],$20);
            if (b and $80) = $80 then
            begin
                inc(FT_Out_Buffer[2*i+16*j],$80);
                inc(FT_Out_Buffer[2*i+1+16*j],$80);
            end;
            b:=b*2;
        end;
    end;
    if (not errorWriteFlag) then
    begin
        errorWriteFlag := Write_USB_Device_Buffer_wErr(FT_HANDLEB,@FT_Out_Buffer,n);
    end;
end;

procedure sspo;
var i:integer;
    b:word;
    n:word;
	byteCnt, byteExpected : Word;
begin
    n:=100;
	  byteCnt := 0;
    byteExpected := n;
    if (not errorWriteFlag) then
    begin
        byteCnt := Read_USB_Device_Buffer(FT_HANDLEB,n);
    end;
    if (byteCnt<>byteExpected) then
    begin
        errorReadFlag:=true;
    end;
	
    b:=0;
    for i:=0 to 15 do
    begin
        b:=b*2;
        if (FT_In_Buffer[i+1+8] and $40) <> 0 then
        begin
            inc(b);
        end;
    end;
    siout:=b;
end;

procedure Spi_comm(comm:byte;param:word);
begin
    Purge_USB_Device_In(FT_HANDLEB);
    Purge_USB_Device_Out(FT_HANDLEB);
    siin[0]:=comm;
    siin[1]:=hi(param);
    siin[2]:=lo(param);
    sspi;
    sspo;
    sleep(20);
end;

procedure ComRead;
begin
    co:=posl.Create(true);
    co.FreeOnTerminate:=true;
    co.Priority:=tpNormal;
    co.Resume;
end;

procedure posl.Execute;
//собственно само чтение массива через порт ADBUS
// Хитрое преобразование считанного буфера FT2232HL в буферный массив изображения
//  из-за особенностей AD9822 считываем сначала старший байт, потом младший, а в delphi наоборот.
//  Используем также  тип integer32, а не word16 из-за переполнения при последующих операциях
var x, y:integer;
    byteCnt : Word;
    byteExpected : Word;
begin
    byteCnt := 0;
    byteExpected := kolbyte;
    if (not errorWriteFlag) then
    begin
        byteCnt := Read_USB_Device_Buffer(FT_HANDLEA,kolbyte);
    end;
    if (byteCnt<>byteExpected) then
    begin
        errorReadFlag:=true;
        if (not errorWriteFlag) then
        begin
            Purge_USB_Device_IN(FT_HANDLEA);
            Purge_USB_Device_OUT(FT_HANDLEA);
        end;
    end
    else begin
        if mBin = 0 then
        begin
            for y:= 0 to mdeltY-1 do
            begin
                for x:=0 to 1499 do
                begin
                    bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[4*x+4+y*6004]);
                    bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[4*x+5+y*6004]);
                    bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[4*x+6+y*6004]);
                    bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[4*x+7+y*6004]);
                end;
            end;
        end
        else begin
            for y:= 0 to mdeltY-1 do
            begin
                for x:=0 to 1498 do
                begin
                    bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
                    bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
                    bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
                    bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+7+y*1504]);
                end;
                x:=1499;
                bufim[2*x+0+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
                bufim[2*x+0+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
                bufim[2*x+1+(2*(y+mYn)+1)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
                bufim[2*x+1+(2*(y+mYn)+0)*3000]:=swap(FT_In_Buffer[x+6+y*1504]);
            end;
        end;
    end;
    
    // discard image if sensorClearing was required (Bias frame before exposure)
    if( sensorClear = true) then
    begin
        imageReady := false;
        sensorClear := false;
    end
    else
    begin
        imageReady := true;
    end;

    cameraState:=cameraIdle;

    // check if we need to update the CameraSetTemp value, i.e. if the set value changed
    // while the sensor was read
    if ( targetTempDirty) then
    begin
        debugToFile('pos1: Setting temperature from cache' + FloatToStr(targetTempCache));
        cameraSetTemp(targetTempCache);
        targetTempDirty := false;
    end;

    if (coolerOnDirty) then
    begin
        debugToFile('pos1: Setting cooler ON/OFF from cache' + BoolToStr(coolerOnCache));
        if (coolerOnCache) then
        begin
            cameraCoolingOn();
        end
        else
        begin
            cameraCoolingOff();
        end;
        coolerOnDirty := false;
    end;

end;

// Заполнение выходного буфера массивом для передачи и размещения байта val по адресу adr в микросхеме AD9822.
// Передача идет в последовательном коде.
procedure AD9822(adr:byte;val:word);
const
    kol = 64;
var
    dan:array[0..kol-1] of byte;
    i:integer;
begin
    //заполняется массив первоначальным значением на выводах порта BDBUS
    fillchar(dan,kol,portfirst);
    for i:=1 to 32 do
    begin
        dan[i]:=dan[i] and $fe;
    end;
    for i:=0 to 15 do
    begin
        dan[2*i+2]:=dan[2*i+2] + 2;
    end;
    if (adr and 4) = 4 then
    begin
        dan[3]:=dan[3]+4;
        dan[4]:=dan[4]+4;
    end;
    if (adr and 2) = 2 then
    begin
        dan[5]:=dan[5]+4;
        dan[6]:=dan[6]+4;
    end;
    if (adr and 1) = 1 then
    begin
        dan[7]:=dan[7]+4;
        dan[8]:=dan[8]+4;
    end;

    if (val and 256) = 256 then
    begin
        dan[15]:=dan[15]+4;
        dan[16]:=dan[16]+4;
    end;
    if (val and 128) = 128 then
    begin
        dan[17]:=dan[17]+4;
        dan[18]:=dan[18]+4;
    end;
    if (val and 64) = 64 then
    begin
        dan[19]:=dan[19]+4;
        dan[20]:=dan[20]+4;
    end;
    if (val and 32) = 32 then
    begin
        dan[21]:=dan[21]+4;
        dan[22]:=dan[22]+4;
    end;
    if (val and 16) = 16 then
    begin
        dan[23]:=dan[23]+4;
        dan[24]:=dan[24]+4;
    end;
    if (val and 8) = 8 then
    begin
        dan[25]:=dan[25]+4;
        dan[26]:=dan[26]+4;
    end;
    if (val and 4) = 4 then
    begin
        dan[27]:=dan[27]+4;
        dan[28]:=dan[28]+4;
    end;
    if (val and 2) = 2 then
    begin
        dan[29]:=dan[29]+4;
        dan[30]:=dan[30]+4;
    end;
    if (val and 1) = 1 then
    begin
        dan[31]:=dan[31]+4;
        dan[32]:=dan[32]+4;
    end;
    if (not errorWriteFlag) then
    begin
        errorWriteFlag := Write_USB_Device_Buffer_wErr(FT_HANDLEB,@dan, kol);
    end;
end;

// Используется 2 режима:
// 1.Цветной без бининга.
// 2.Ч/Б с бинингом 2*2.
// Особенностью матрицы ICX453 является то, что горизонтальный регистр имеет удвоенную емкость и 
// при одном шаге вертикального сдвига в горизонтальный регистр "падает" сразу пара строк,
// поэтому количество строк для этих двух режимиов одинаковое.

// Заполнение выходного буфера массивом и собственно сама операция чтения кадра в 1 режиме
procedure readframe;
begin
    debugToFile('readframe');

    cameraState := cameraReading;
    Purge_USB_Device_IN(FT_HANDLEA);
    // Purge_USB_Device_OUT(FT_HANDLEB);
    comread;
    Spi_comm($1b,0);
end;

// -------------------------------------------------------------------------------
// Interface part
// -------------------------------------------------------------------------------

// Set camera gain, return bool result
function cameraSetGain (val : integer) : WordBool; stdcall; export;
begin
    //усиление AD9822
    AD9822(3,val);
    Result :=true;
end;

//Set camera offset, return bool result
function cameraSetOffset (val : integer) : WordBool; stdcall; export;
var x : integer;
begin
    debugToFile('cameraSetOffset');

    x:=abs(2*val);
    if val < 0 then
    begin
        x:=x+256;
    end;
    //смещение AD9822
    AD9822(6,x);
    Result :=true;
end;

// Connect camera, return bool result
// Опрос подключенных устройств и инициализация AD9822
function cameraConnect () : WordBool; stdcall; export;
var FT_flag : boolean;
begin
    FT_Enable_Error_Report := true;
    FT_flag := true;
    errorWriteFlag := false;
    sensorTempCache := 0;
    targetTempCache := 0;
    targetTempDirty := false;
    coolerOnCache := false;
    coolerOnDirty := false;
    coolerPowerCache := 0;
    firmwareVersionCache := 0;

    debugToFile('cameraConnect');

    if (FT_flag) then
    begin
        if Open_USB_Device_By_Serial_Number(FT_HANDLEA,'CAM86A') <> FT_OK then
        begin
            FT_flag := false;
        end;
    end;
    if (FT_flag) then
    begin
        if Open_USB_Device_By_Serial_Number(FT_HANDLEB,'CAM86B')  <> FT_OK then
        begin
            FT_flag := false;
        end;
    end;
    if (FT_flag) then
    begin
        // BitMode for B-channel
        if Set_USB_Device_BitMode(FT_HANDLEB,$bf, $4)  <> FT_OK then
        begin
            FT_flag := false;
        end;
    end;
    if (FT_flag) then
    begin
        //speed = spusb
        FT_Current_Baud:=spusb;
        Set_USB_Device_BaudRate(FT_HANDLEB);

        //максимальное быстродействие
        Set_USB_Device_LatencyTimer(FT_HANDLEB,2);
        Set_USB_Device_LatencyTimer(FT_HANDLEA,2);
        Set_USB_Device_TimeOuts(FT_HANDLEA,6000,100);
        Set_USB_Device_TimeOuts(FT_HANDLEB,100,100);
        Set_USB_Parameters(FT_HANDLEA,65536,0);

        Purge_USB_Device_IN(FT_HANDLEA);
        Purge_USB_Device_OUT(FT_HANDLEA);
        Purge_USB_Device_IN(FT_HANDLEB);
        Purge_USB_Device_OUT(FT_HANDLEB);

        adress:=0;

        //режим AD9822 - канал G,4 вольта опорность, CDS режим
        AD9822(0,$d8);
        AD9822(1,$a0);
    
        CameraSetGain(0);
        //усиление устанавливается такое. что не переполняется АЦП
        CameraSetOffset(-6);

        sleep(100);
        //send init command
        Spi_comm($db,0);
        sleep(100);
        
        //убрать 2 байта, возникших после reset
        Purge_USB_Device_IN(FT_HANDLEA);
        mBin:=0;
    end;
    isConnected := FT_flag;
    errorReadFlag := false;
    cameraState := cameraIdle;
    imageReady := false;
    if(FT_flag = false) then
    begin
        cameraState := cameraError;
    end;
    Result := isConnected;
end;

//Disconnect camera, return bool result
function cameraDisconnect (): WordBool; stdcall; export;
var FT_OP_flag : boolean;
begin
    debugToFile('cameraDisconnect');

    FT_OP_flag := true;
    //закрытие устройств
    if Close_USB_Device(FT_HANDLEA) <> FT_OK then
    begin
        FT_OP_flag := false;
    end;
    if Close_USB_Device(FT_HANDLEB) <> FT_OK then
    begin
        FT_OP_flag := false;
    end;
    isConnected := not FT_OP_flag;
    Result:= FT_OP_flag;
end;

// Check camera connection, return bool result}
function cameraIsConnected () : WordBool; stdcall; export;
begin
    debugToFile('cameraIsConnected');
    Result := isConnected;
end;

procedure ExposureTimerTick(TimerID, Msg: Uint; dwUser, dw1, dw2: DWORD); stdcall;
begin
    debugToFile('ExposureTimerTick:' +
                'exp_time_left=' + IntToStr(exposure_time_left) +
                ', exp_time_loop_counter=' + IntToStr(exposure_time_loop_counter));

    if exposure_time_loop_counter > 0 then
    begin
        // reduce the counter for the loop that just finished
        exposure_time_loop_counter := exposure_time_loop_counter - 1;

        // restart the timer if needed
        if exposure_time_loop_counter > 0 then
        begin
            debugToFile('ExposureTimerTick: restarting timer' +
            ', exp_time_left=' + IntToStr(exposure_time_left) +
            ', exp_time_loop_counter=' + IntToStr(exposure_time_loop_counter));
            ExposureTimer := TimeSetEvent(eposure_time_rollover, 100, @ExposureTimerTick, 0, TIME_ONESHOT);
            Exit;
        end
        else begin
            if exposure_time_left > 0 then
            begin
                debugToFile('ExposureTimerTick: final restarting of timer' +
                ', exp_time_left=' + IntToStr(exposure_time_left) +
                ', exp_time_loop_counter=' + IntToStr(exposure_time_loop_counter));
                ExposureTimer := TimeSetEvent(exposure_time_left, 100, @ExposureTimerTick, 0, TIME_ONESHOT);
                Exit;
            end;
        end;
    end;

    // if we get here then we are finished
    debugToFile('ExposureTimerTick: final read');
    readframe;
end;

procedure cameraSensorClearFull();
var expoz:integer;
begin
    debugToFile('cameraSensorClearFull');

    errorReadFlag := false;
    imageReady := false;
    mYn:=0;
    Spi_comm($4b,mYn);
    mdeltY:=CameraHeight div 2;
    Spi_comm($5b,mdeltY);

    // use 2x2 binning to increase the reading speed
    // the image will be deleted anyway
    kolbyte:=mdeltY*3008;
    //bining
    Spi_comm($8b,1);
    mBin:=1;

    expoz := 0; // zero exposure
    Spi_comm($6b,expoz);

    cameraState := cameraExposing;

    eexp:=0;
    readframe;

    // wait until the bias frame has been read - we will discard the data
    // This will lock this main thread for a short time... not sure if this is a good thing?
    // this seems to take 1600 ms
    while (sensorClear = True) do
    begin
      sleep(10);
    end;

    // now exit to do proper exposure
end;

function cameraStartExposure (Bin, StartX, StartY, NumX, NumY : integer; Duration : double; light : WordBool) : WordBool; stdcall; export;
var expoz:integer;
begin
    debugToFile('cameraStartExposure');
    
    if (sensorClear) then
    begin
      cameraSensorClearFull;
    end;

    errorReadFlag := false;
    imageReady := false;
    
    mYn:=StartY div 2;
    Spi_comm($4b,mYn);
    mdeltY:=NumY div 2;
    Spi_comm($5b,mdeltY);

    if bin = 2 then
    begin
        kolbyte:=mdeltY*3008;
        //bining
        Spi_comm($8b,1);
        mBin:=1;
    end
    else begin
        kolbyte:=mdeltY*12008;
        //no bining
        Spi_comm($8b,0);
        mBin:=0;
    end;

    expoz:=round(Duration*1000);
    if expoz > 1000 then
    begin
        expoz:=1001;
    end;
    Spi_comm($6b,expoz);

    cameraState := cameraExposing;
    if Duration > 1.0 then
    begin
        //shift3
        Spi_comm($2b,0);
        sleep(40);
        //clear frame
        Spi_comm($cb,0);
        // for time of clear frame
        sleep(180);
        //off 15v
        Spi_comm($3b,0);
        eexp:=round(1000*(Duration-1.2));
        if eexp < 0 then
        begin
            eexp := 0;
        end;

        // calculate the exposure time with repetitions of the timing loop
        exposure_time_left := eexp mod eposure_time_rollover;
        exposure_time_loop_counter := eexp div eposure_time_rollover;

        debugToFile('----------------');
        debugToFile('cameraStartExposure: Duration=' + FloatToStr(Duration) +
                    ', eexp=' + IntToStr(eexp) +
                    ', exp_time_left=' + IntToStr(exposure_time_left) +
                    ', exp_time_loop_counter=' + IntToStr(exposure_time_loop_counter) +
                    ', eposure_time_rollover=' + IntToStr(eposure_time_rollover));

        // start exposure while paying attention to the repetitions of the timing loop
        if exposure_time_loop_counter > 0 then
        begin
            ExposureTimer := TimeSetEvent(eposure_time_rollover, 100, @ExposureTimerTick, 0, TIME_ONESHOT);
        end
        else begin
            if exposure_time_left > 0 then
            begin
                ExposureTimer := TimeSetEvent(exposure_time_left, 100, @ExposureTimerTick, 0, TIME_ONESHOT);
            end
            else begin
                eexp := 0;
                readframe;
            end;
        end;

    end
    else begin
        eexp:=0;
        readframe;
    end;
    Result := true;
end;

function cameraStopExposure : WordBool; stdcall; export;
begin
    debugToFile('cameraStopExposure');

    TimeKillEvent(ExposureTimer);
    if (cameraState = cameraExposing) then
    begin
        readframe;
    end;
    Result := true;
end;

//Get camera state, return int result
function cameraGetCameraState : integer; stdcall; export;
begin
    debugToFile('cameraGetCameraState');

    if (not errorWriteFlag) then
    begin
      Result := cameraState
    end
    else begin
        Result := cameraError;
    end;
end;

//Check ImageReady flag, is image ready for transfer - transfer image to driver and return bool ImageReady flag
function cameraGetImageReady : WordBool; stdcall; export;
begin
    debugToFile('cameraGetImageReady');

    Result := imageReady;
end;

//Get back pointer to image
function cameraGetImage : dword; stdcall; export;
begin
    debugToFile('cameraGetImage');

    cameraState:=cameraDownload;
    cameraState:=cameraIdle;
    Result := dword(@bufim);
end;

//Get camera error state, return bool result
function cameraGetError : integer; stdcall; export;
var res : integer;
begin
    debugToFile('cameraGetError');

    res:=0;
    if (errorWriteFlag) then res :=res+2;
    if (errorReadFlag) then res :=res+1;
    Result:=res;
end;

function cameraGetTemp (): double; stdcall; export;
var temp : double;
begin
    debugToFile('cameraGetTemp');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := sensorTempCache;
    end
    else
    begin
        Spi_comm($bf,0);
        temp := (siout - TemperatureOffset) / 10.0;
        if ((temp > MaxErrTemp) or (temp < MinErrTemp)) then
        begin
            temp := sensorTempCache;
        end;
        sensorTempCache := temp;
        Result := temp;
    end;
end;

function cameraSetTemp(temp : double): WordBool; stdcall; export;
var d0:word;
begin
    debugToFile('cameraSetTemp');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        targetTempCache := temp;
        targetTempDirty := true;
        debugToFile('cameraSetTemp: Caching temperature ' + FloatToStr(targetTempCache));
    end
    else
    begin
        d0 := TemperatureOffset + round(temp*10);
        Spi_comm($ab,d0);
        targetTempDirty := false;
        debugToFile('cameraSetTemp: setting temperature ' + FloatToStr(targetTempCache));
    end;
    
    Result := true;
end;

function cameraGetSetTemp (): double; stdcall; export;
var temp : double;
begin
    debugToFile('cameraGetSetTemp');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := targetTempCache;
    end
    else
    begin
      Spi_comm($be,0);
      temp := (siout - TemperatureOffset) / 10.0;
      if ((temp > MaxErrTemp) or (temp < MinErrTemp)) then
      begin
          temp := targetTempCache;
      end;
      targetTempCache := temp;
      Result := temp;
    end;
end;

function cameraCoolingOn (): WordBool; stdcall; export;
begin
    debugToFile('cameraCoolingOn');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        debugToFile('cameraCoolingOn: Caching cooler on value');
        coolerOnCache := true;
        coolerOnDirty := true;
    end
    else
    begin
        Spi_comm($9b,1);
    end;

    Result := true;
end;

function cameraCoolingOff (): WordBool; stdcall; export;
begin
    debugToFile('cameraCoolingOff');
    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        debugToFile('cameraCoolingOn: Caching cooler off value');
        coolerOnCache := false;
        coolerOnDirty := true;
    end
    else
    begin
      Spi_comm($9b,0);
    end;
    
    Result := true;
end;

function cameraGetCoolerOn (): WordBool; stdcall; export;
begin
    debugToFile('cameraGetCoolerOn');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := coolerOnCache;
    end
    else
    begin
        Spi_comm($bd,0);
        if (siout = TRUE_INV_PROT) then
        begin
            coolerOnCache := true;
            Result := true;
        end
        else if (siout = FALSE_INV_PROT) then
        begin
            coolerOnCache := false;
            Result := false;
        end
        else begin
            Result := coolerOnCache;
        end;
    end;
end;

function cameraGetCoolerPower (): double; stdcall; export;
var power : double;
begin
    debugToFile('cameraGetCoolerPower');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := coolerPowerCache;
    end
    else
    begin
        Spi_comm($bc,0);
        if (( siout shr 8) = (HIGH_MASK_PROT shr 8)) then
        begin
            power := (siout and $00ff) / 2.55;
        end
        else begin
            power := coolerPowerCache;
        end;
        coolerPowerCache := power;
        Result := power;
    end;
end;

function cameraSetReadingTime(val: integer)  : WordBool; stdcall; export;
begin
    debugToFile('cameraSetReadingTime');

    Spi_comm($eb, val);
    Result := true;
end;

function cameraGetFirmwareVersion : byte; stdcall; export;
begin
    debugToFile('cameraGetFirmwareVersion');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := firmwareVersionCache;
    end
    else
    begin
        Spi_comm($bb,0);
        firmwareVersionCache := siout And $ff;
        Result := firmwareVersionCache;
    end;
end;

function cameraSetCoolerDuringReading(val: integer) : Wordbool; stdcall; export;
begin
    debugToFile('cameraSetCoolerDuringReading');

    Spi_comm($fb, val);
    Result := true;
end;

function cameraGetLLDriverVersion : byte; stdcall; export
begin
    debugToFile('cameraGetLLDriverVersion');

    Result := softwareLLDriverVersion;
end;

function cameraSetBiasBeforeExposure(val: Wordbool) : WordBool; stdcall; export;
begin
    debugToFile('cameraSetBiasBeforeExposure');

    sensorClear := val;
    Result := true;
end;

function cameraGetTempDHT (): double; stdcall; export
begin
    debugToFile('cameraGetTempDHT');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := tempDHTCache;
    end
    else
    begin
      Spi_comm($f1,0);
      tempDHTCache := (siout - TemperatureOffset) / 10.0;
      Result := tempDHTCache;
    end
end;

function cameraGetHumidityDHT (): double; stdcall; export
begin
    debugToFile('cameraGetHumidityDHT');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := humidityDHTCache;
    end
    else
    begin
      Spi_comm($f2,0);
      humidityDHTCache := (siout) / 10.0;
      Result := humidityDHTCache;
    end
end;

function cameraSetCoolingStartingPowerPercentage(val: integer)  : WordBool; stdcall; export;
begin
    debugToFile('cameraSetCoolingStartingPowerPercentage');

    Spi_comm($0a, val);
    Result := true;
    CoolingStartingPowerPercentageCache := val;
end;

function cameraSetCoolingMaximumPowerPercentage(val: integer)  : WordBool; stdcall; export;
begin
    debugToFile('cameraSetCoolingMaximumPowerPercentage');

    Spi_comm($1a, val);
    CoolingMaximumPowerPercentageCache := val;
    Result := true;
end;

function cameraGetCoolingStartingPowerPercentage : integer; stdcall; export;
begin
    debugToFile('cameraGetCoolingStartingPowerPercentage');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := CoolingStartingPowerPercentageCache;
    end
    else
    begin
        Spi_comm($ba,0);
        CoolingStartingPowerPercentageCache := siout;
        Result := CoolingStartingPowerPercentageCache;
    end;
end;

function cameraGetCoolingMaximumPowerPercentage : integer; stdcall; export;
begin
    debugToFile('cameraGetCoolingMaximumPowerPercentage');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        Result := CoolingMaximumPowerPercentageCache;
    end
    else
    begin
        Spi_comm($b9,0);
        CoolingMaximumPowerPercentageCache := siout;
        Result := CoolingMaximumPowerPercentageCache;
    end;
end;

function cameraSetPIDproportionalGain(val: double)  : WordBool; stdcall; export;
begin
    debugToFile('cameraSetPIDproportionalGain');

    Spi_comm($2a, trunc(val * 1000.0));
    KpCache := val;

    debugToFile('cameraSetPIDproportionalGain: value=' + FloatToStr(val));

    Result := true;
end;

function cameraGetPIDproportionalGainOld : double; stdcall; export;
var temp : integer;
begin
    debugToFile('cameraGetPIDproportionalGainOld');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        debugToFile('cameraGetPIDproportionalGain: Using cached value');
        Result := KpCache / 1000.0;
    end
    else
    begin
        Spi_comm($b8,0);
        temp := siout;
        KpCache := temp;
        debugToFile('cameraGetPIDproportionalGain: value=' + IntToStr(temp) + ', Result=' + FloatToStr(temp / 1000.0));
        Result := temp / 1000.0;
    end;
end;

function cameraGetPIDproportionalGainLow : word; stdcall; export;
begin
    debugToFile('cameraGetPIDproportionalGainLow');

    Spi_comm($b8,0);
    Result := siout;
end;

function cameraGetPIDproportionalGainHigh : word; stdcall; export;
begin
    debugToFile('cameraGetPIDproportionalGainHigh');

    Spi_comm($b7,0);
    Result := siout;
end;

function cameraGetPIDproportionalGain : double; stdcall; export;
var
    temp : array[0..3] of byte;
    temp_low : word;
    temp_high : word;
    temp_single : single;
begin
    debugToFile('cameraGetPIDproportionalGain');

    if ((cameraState = cameraReading) or (cameraState = cameraDownload)) then
    begin
        debugToFile('cameraGetPIDproportionalGain: Using cached value');
        Result := KpCache;
    end
    else
    begin
        // this is a pain, AVR compiler fails to convert float to integers correctly
        // under some conditions.
        // for example
        // float x = 0.4*1000.0 equals 400.0 while
        // float a = 0.4; 
        // float x = a * 1000.0 equals 100.0
        // ??????
        // we read 4-byte long byte array and convert it to a floating point
        temp_low := cameraGetPIDproportionalGainLow();
        temp_high := cameraGetPIDproportionalGainHigh();
        temp[0] := temp_low And $FF;
        temp[1] := temp_low Shr 8;
        temp[2] := temp_high And $FF;
        temp[3] := temp_high Shr 8;

        // convert the byte array to single float (4 bytes long)
        Move(temp, temp_single, SizeOf( single));

        debugToFile('cameraGetPIDproportionalGain: temp_low=' + IntToStr(temp_low) + ', temp_high=' + IntToStr(temp_high) +
                    ', raw value=' + IntToStr(temp[0]) + ', ' + IntToStr(temp[1]) + ', ' +
                    IntToStr(temp[2]) + ', ' + IntToStr(temp[3]) + '; Result=' + FloatToStr(temp_single));

        KpCache := temp_single;
        Result := KpCache;
    end;
end;

exports cameraGetLLDriverVersion;
exports cameraConnect;
exports cameraDisconnect;
exports cameraIsConnected;
exports cameraStartExposure;
exports cameraStopExposure;
exports cameraGetCameraState;
exports cameraGetImageReady;
exports cameraGetImage;
exports cameraSetGain;
exports cameraSetOffset;
exports cameraGetError;
exports cameraGetTemp;
exports cameraSetTemp;
exports cameraGetSetTemp;
exports cameraCoolingOn;
exports cameraCoolingOff;
exports cameraGetCoolerOn;
exports cameraGetCoolerPower;
exports cameraSetReadingTime;
exports cameraGetFirmwareVersion;
exports cameraSetCoolerDuringReading;
exports cameraSetBiasBeforeExposure;
exports cameraGetTempDHT;
exports cameraGetHumidityDHT;
exports cameraSetCoolingStartingPowerPercentage;
exports cameraSetCoolingMaximumPowerPercentage;
exports cameraSetPIDproportionalGain;
exports cameraGetPIDproportionalGain;
exports cameraGetCoolingStartingPowerPercentage;
exports cameraGetCoolingMaximumPowerPercentage;

begin

end.


