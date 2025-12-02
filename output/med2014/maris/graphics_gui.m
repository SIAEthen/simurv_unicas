function varargout = graphics_gui(varargin)
% GRAPHICS_GUI M-file for graphics_gui.fig
%      GRAPHICS_GUI, by itself, creates a new GRAPHICS_GUI or raises the existing
%      singleton*.
%
%      H = GRAPHICS_GUI returns the handle to a new GRAPHICS_GUI or the handle to
%      the existing singleton*.
%
%      GRAPHICS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRAPHICS_GUI.M with the given input arguments.
%
%      GRAPHICS_GUI('Property','Value',...) creates a new GRAPHICS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before graphics_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to graphics_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help graphics_gui

% Last Modified by GUIDE v2.5 30-Jan-2014 13:03:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @graphics_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @graphics_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before graphics_gui is made visible.
function graphics_gui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to graphics_gui (see VARARGIN)

% Choose default command line output for graphics_gui
handles.output = hObject;

% Import from Workspace
a=evalin('base','who');

if( ismember('time',a)) 
    
    handles.time        = evalin('base','time');
    handles.pos_v       = evalin('base','PosV');
    handles.ori_v       = evalin('base','OriV');
    handles.velLin_v    = evalin('base','VelLinV');
    handles.velAng_v    = evalin('base','VelAngV');
    handles.pos_j       = evalin('base','PosJ');
    handles.vel_j       = evalin('base','VelJ');
    handles.end_j       = evalin('base','PoseEnd');
    handles.torque_arm  = evalin('base','torqueM');
    handles.parameters  = evalin('base','Theta');

    handles.pos_v_des   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des   = evalin('base','dEta_d(:,1:3)');
    handles.pos_j_des   = evalin('base','Pos_jdes');
    handles.vel_j_des   = evalin('base','Vel_jdes');
    
    handles.thrust      = evalin('base','ForceV(:,1:3)');
    handles.torque      = evalin('base','ForceV(:,4:6)');    
    
    handles.n_v = 9;
    handles.n_a = 6;
    
    for i = 1: length(handles.time)
        handles.err_pos_v(i,:) = norm(handles.pos_v_des(i,:)-handles.pos_v(i,:) );
    end
    
    axes(handles.axes1);
    plot(handles.time,handles.pos_v,handles.time,handles.pos_v_des,'r--');
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.err_pos_v,'r');%,handles.time,handles.t_position2,'r--');
    grid on;
    xlabel('Time (s)'); ylabel('Error pos Vehicle (m)');legend('Position'); set(legend,'Visible','off');

    set(handles.text1,'String','Positions')
    set(handles.radiobutton1,'Value',1)
    set(handles.radiobutton3,'Value',0)
    set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
    set(handles.pushbutton18,'Backgroundcolor',[0.8 0.8 0.8])
    handles.v_a = 1;
    handles.current_joint =1;
    handles.compare = 0;
    handles.compare_now = 0;
   
end    
    handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes graphics_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = graphics_gui_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(~, ~, handles) %#ok<*DEFNU>
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.v_a==1 && handles.compare==0 && handles.compare_now==0)
    axes(handles.axes1);
    plot(handles.time,handles.pos_v,handles.time,handles.pos_v_des,'r--');
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.ori_v);%,handles.time,handles.t_position2,'r--');
    grid on;
    xlabel('Time (s)'); ylabel('orien Vehicle YPR(rad)');legend('orien','Trajectory'); set(legend,'Visible','off')
    text_my ='Pos & Angle';
elseif(handles.v_a==0 && handles.compare==0 && handles.compare_now==0 )%&& handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time,handles.pos_j(:,handles.current_joint),'b',handles.time,handles.pos_j_des(:,handles.current_joint),'r--');
    grid on;
    ylabel('Position Joint (rad)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.pos_j(:,handles.current_joint)-handles.pos_j_des(:,handles.current_joint),'r');
    grid on;
    xlabel('Time (s)'); ylabel('Position error (rad)');legend('Position'); set(legend,'Visible','off')
    text_my =strcat('Pos & Angle Joint ',num2str(handles.current_joint));
elseif(handles.v_a==1 && ( handles.compare==1 || handles.compare_now==1 ))
    axes(handles.axes1);
    plot(handles.time1,handles.pos_v1,'b',handles.time1,handles.pos_v_des1,'g--');
    hold on 
    plot(handles.time2,handles.pos_v2,'r',handles.time2,handles.pos_v_des2,'y--');
    grid on;hold off 
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time1,handles.ori_v1,'b');%,handles.time,handles.t_position2,'r--');
    hold on
    plot(handles.time2,handles.ori_v2,'r');
    grid on;hold off 
    xlabel('Time (s)'); ylabel('orien Vehicle YPR (rad)');legend('orien','Trajectory'); set(legend,'Visible','off')
    text_my =strcat('Pos & Angle',{' red '},handles.name1 ,{'& blue '},handles.name2);
elseif(handles.v_a==0 && ( handles.compare==1 || handles.compare_now==1 ))% && handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time1,handles.pos_j1(:,handles.current_joint),'b',handles.time1,handles.pos_j_des1(:,handles.current_joint),'g--');
    hold on 
    plot(handles.time2,handles.pos_j2(:,handles.current_joint),'r',handles.time2,handles.pos_j_des2(:,handles.current_joint),'y--');
    grid on;hold off 
    ylabel('Position Joint (rad)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time1,handles.pos_j1(:,handles.current_joint)-handles.pos_j_des1(:,handles.current_joint),'b');
    hold on
    plot(handles.time2,handles.pos_j2(:,handles.current_joint)-handles.pos_j_des2(:,handles.current_joint),'r');
    grid on;hold off 
    xlabel('Time (s)'); ylabel('Position error (rad)');legend('Position'); set(legend,'Visible','off')
    text_my =strcat('Pos & Angle Joint ',num2str(handles.current_joint),{' blue '},handles.name1 ,{'& red '},handles.name2);
end

set(handles.text1,'String',text_my)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(~, ~, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.v_a==1 && handles.compare==0 && handles.compare_now==0)
    axes(handles.axes1);
    plot(handles.time,handles.velLin_v,handles.time,handles.vel_v_des,'r--');
    grid on;
    ylabel('Velocities Vehicle (m/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.velAng_v);
    grid on;
    xlabel('Time (s)'); ylabel('Angular Velocities YPR (rad/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off')
    text_my ='Velocities';
elseif(handles.v_a==0 && handles.compare==0 && handles.compare_now==0)%&& handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time,handles.vel_j(:,handles.current_joint),'b',handles.time,handles.vel_j_des(:,handles.current_joint),'r--');
    grid on;
    ylabel('Velocities Joint (rad/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.vel_j(:,handles.current_joint)-handles.vel_j_des(:,handles.current_joint),'r');
    grid on;
    xlabel('Time (s)'); ylabel('Velocities error (rad/s)');legend('Velocities'); set(legend,'Visible','off')
    text_my =strcat('Velocities Joint ',num2str(handles.current_joint));
elseif(handles.v_a==1 && ( handles.compare==1 || handles.compare_now==1 ))
    axes(handles.axes1);
    plot(handles.time1,handles.velLin_v1,'b',handles.time1,handles.vel_v_des1,'g--');
    grid on;
    hold on
    plot(handles.time2,handles.velLin_v2,'r',handles.time2,handles.vel_v_des2,'y--');hold off 
    ylabel('Velocities Vehicle (m/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time1,handles.velLin_v1-handles.vel_v_des1,'b');
    hold on
    plot(handles.time2,handles.velLin_v2-handles.vel_v_des2,'r');
    grid on;hold off 
    xlabel('Time (s)'); ylabel('Velocities error (m/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off')
    text_my =strcat('Velocities',{' blue '},handles.name1 ,{'& red '},handles.name2);
elseif(handles.v_a==0 && ( handles.compare==1 || handles.compare_now==1 ))% && handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time1,handles.vel_j1(:,handles.current_joint),'b',handles.time1,handles.vel_j_des1(:,handles.current_joint),'g--');
    grid on;
    hold on
    plot(handles.time2,handles.vel_j2(:,handles.current_joint),'r',handles.time2,handles.vel_j_des2(:,handles.current_joint),'y--');
    ylabel('Velocities Joint (rad/s)');legend('Velocities','Trajectory'); set(legend,'Visible','off');hold off 
    axes(handles.axes2);
    plot(handles.time1,handles.vel_j1(:,handles.current_joint)-handles.vel_j_des1(:,handles.current_joint),'b');
    grid on;
    hold on
    plot(handles.time2,handles.vel_j2(:,handles.current_joint)-handles.vel_j_des2(:,handles.current_joint),'r');hold off 
    xlabel('Time (s)'); ylabel('Velocities error (rad/s)');legend('Velocities'); set(legend,'Visible','off')
    text_my =strcat('Velocities Joint ',num2str(handles.current_joint),{' blue '},handles.name1 ,{'& red '},handles.name2);
end

set(handles.text1,'String',text_my )
    

% % --- Executes on button press in pushbutton3.
% function pushbutton3_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton3 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
%     axes(handles.axes1);
%     plot(handles.time,handles.acc1,'b',handles.time,handles.t_acc1,'r--');
%     grid on;
%     ylabel('Accelerations 1 (rad/s^2)');legend('Accelerations','Trajectory'); set(legend,'Visible','off')
%     axes(handles.axes2);
%     plot(handles.time,handles.acc2,'b',handles.time,handles.t_acc2,'r--');
%     grid on;
%     xlabel('Time (s)'); ylabel('Accelerations 2 (rad/s^2)');legend('Accelerations','Trajectory'); set(legend,'Visible','off')
% 
%     set(handles.text1,'String','Accelerations')

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(~, ~, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
 if(handles.v_a==1 && handles.compare==0 && handles.compare_now==0)
    axes(handles.axes1);
    plot(handles.time,handles.thrust);
    grid on;
    ylabel('Thrust (N)');
    axes(handles.axes2);
    plot(handles.time,handles.torque);
    grid on;
    xlabel('Time (s)'); ylabel('Torque (Nm)');
    text_my = 'Forces';
 elseif(handles.v_a==0 && handles.compare==0 && handles.compare_now==0)%&& handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time,handles.torque_arm(:,handles.current_joint),'b');
    grid on;
    ylabel('Torque arm (Nm)');
    text_my =strcat('Forces Joint ',num2str(handles.current_joint));
    axes(handles.axes2);
    plot(0,0)
 elseif(handles.v_a==1 && ( handles.compare==1 || handles.compare_now==1 ))
    axes(handles.axes1);
    plot(handles.time1,handles.thrust1(:,3),'b');
    grid on;
    hold on
    plot(handles.time2,handles.thrust2(:,3),'r');
    ylabel('Thrust (N)');hold off 
    axes(handles.axes2);
    plot(handles.time1,handles.torque1,'b');
    grid on;
    hold on
    plot(handles.time2,handles.torque2,'r');
    xlabel('Time (s)'); ylabel('Torque (Nm)');hold off 
    text_my = strcat('Forces',{' blue '},handles.name1 ,{'& red '},handles.name2);
 elseif(handles.v_a==0 && ( handles.compare==1 || handles.compare_now==1 ))% && handles.w == 1 )
    axes(handles.axes1);
    plot(handles.time1,handles.torque_arm1(:,handles.current_joint),'b');
    grid on;
    hold on
    plot(handles.time2,handles.torque_arm2(:,handles.current_joint),'r');
    ylabel('Torque arm (Nm)');hold off 
    axes(handles.axes2);
    plot(0,0)
    text_my =strcat('Forces Joint ',num2str(handles.current_joint),{' blue '},handles.name1 ,{'& red '},handles.name2);
 end
 
 set(handles.text1,'String',text_my)

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(~, ~, ~)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(~, ~, ~)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(~, ~, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%option_gui
name = strcat('report/',handles.name_file,'.mat');
assignin('base','name',name);

if handles.compare_now == 0
    
    evalin('base', 'save(name)');

elseif handles.compare_now == 1

    assignin('base','file_name','matlab.mat');
    evalin('base','copyfile(file_name,name)');
    evalin('base','delete matlab.mat');
    evalin('base','clear file_name ');

end

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(~, ~, ~)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, ~, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str= get(hObject, 'String');
val= get(hObject,'Value');

switch str{val};
    case 'Joint 1'
        i= 1;
    case 'Joint 2'
        i= 2;
    case 'Joint 3'
        i= 3;
    case 'Joint 4'
        i= 4;
    case 'Joint 5'
        i= 5;
    case 'Joint 6'
        i= 6;
    case 'Joint 7'
        i= 7;
end

handles.current_joint = i;
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, ~, ~)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, ~, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
evalin('base','clear')
evalin('base','[FileName,PathName,FilterIndex] = uigetfile();');
evalin('base','load(fullfile(PathName,FileName));');
 set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
% Import from Workspace
a=evalin('base','who');

if( ismember('time',a)) 
    
    handles.time        = evalin('base','time');
    handles.pos_v       = evalin('base','PosV');
    handles.ori_v       = evalin('base','OriV');
    handles.velLin_v    = evalin('base','VelLinV');
    handles.velAng_v    = evalin('base','VelAngV');
    handles.pos_j       = evalin('base','PosJ');
    handles.vel_j       = evalin('base','VelJ');
    handles.end_j       = evalin('base','PoseEnd');
    handles.torque_arm  = evalin('base','torqueM');
    handles.parameters  = evalin('base','Theta');

    handles.pos_v_des   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des   = evalin('base','Pos_jdes');
    handles.vel_j_des   = evalin('base','Vel_jdes');
    
    handles.thrust      = evalin('base','ForceV(:,1:3)');
    handles.torque      = evalin('base','ForceV(:,4:6)'); 
    
    for i = 1: length(handles.time)
        handles.err_pos_v(i,:) = norm(handles.pos_v_des(i,:)-handles.pos_v(i,:) );
    end

    axes(handles.axes1);
    plot(handles.time,handles.pos_v,handles.time,handles.pos_v_des,'r--');
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.err_pos_v,'r');%,handles.time,handles.t_position2,'r--');
    grid on;
    xlabel('Time (s)'); ylabel('Error pos Vehicle (m)');legend('Position'); set(legend,'Visible','off')

    set(handles.text1,'String','Positions')
    set(handles.radiobutton1,'Value',1)
    set(handles.radiobutton3,'Value',0)
    set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
    set(handles.pushbutton18,'Backgroundcolor',[0.8 0.8 0.8])
    handles.v_a = 1;
    handles.current_joint =1;
    handles.compare = 0;
    handles.compare_now = 0;
   
end 

guidata(hObject, handles);

% --- Executes on button press in pushbutton14.
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% % --- Executes on button press in pushbutton14.
% function pushbutton14_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton14 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, ~, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.compare == 1
    handles.compare =0;
    set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
elseif handles.compare == 0

    evalin('base','clear')
    evalin('base','[FileName,PathName,FilterIndex] = uigetfile();');
    evalin('base','load(fullfile(PathName,FileName));');

    handles.name1        = evalin('base','FileName');
    handles.time1        = evalin('base','time');
    handles.pos_v1       = evalin('base','PosV');
    handles.ori_v1       = evalin('base','OriV');
    handles.velLin_v1    = evalin('base','VelLinV');
    handles.velAng_v1    = evalin('base','VelAngV');
    handles.pos_j1       = evalin('base','PosJ');
    handles.vel_j1       = evalin('base','VelJ');
    handles.end_j1       = evalin('base','PoseEnd');
    handles.torque_arm1  = evalin('base','torqueM');
    handles.parameters1  = evalin('base','Theta');

    handles.pos_v_des1   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des1   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des1   = evalin('base','Pos_jdes');
    handles.vel_j_des1   = evalin('base','Vel_jdes');

    handles.thrust1      = evalin('base','ForceV(:,1:3)');
    handles.torque1      = evalin('base','ForceV(:,4:6)'); 

    for i = 1: length(handles.time1)
        handles.err_pos_v1(i,:) = norm(handles.pos_v_des1(i,:)-handles.pos_v1(i,:) );
    end
  
    evalin('base','clear')
    evalin('base','[FileName,PathName,FilterIndex] = uigetfile();');
    evalin('base','load(fullfile(PathName,FileName));');

    handles.name2        = evalin('base','FileName');
    handles.time2        = evalin('base','time');
    handles.pos_v2       = evalin('base','PosV');
    handles.ori_v2       = evalin('base','OriV');
    handles.velLin_v2    = evalin('base','VelLinV');
    handles.velAng_v2    = evalin('base','VelAngV');
    handles.pos_j2       = evalin('base','PosJ');
    handles.vel_j2       = evalin('base','VelJ');
    handles.end_j2       = evalin('base','PoseEnd');
    handles.torque_arm2  = evalin('base','torqueM');
    handles.parameters2  = evalin('base','Theta');

    handles.pos_v_des2   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des2   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des2   = evalin('base','Pos_jdes');
    handles.vel_j_des2   = evalin('base','Vel_jdes');
    
    handles.thrust2      = evalin('base','ForceV(:,1:3)');
    handles.torque2      = evalin('base','ForceV(:,4:6)'); 

    for i = 1: length(handles.time2)
        handles.err_pos_v2(i,:) = norm(handles.pos_v_des2(i,:)-handles.pos_v2(i,:) );
    end
    
    set(handles.pushbutton15,'Backgroundcolor',[1 0 0])
    handles.compare = 1;
    
    axes(handles.axes1);
    plot(handles.time1,handles.pos_v1,'b',handles.time1,handles.pos_v_des1,'g--');
    hold on
    plot(handles.time2,handles.pos_v2,'r',handles.time2,handles.pos_v_des2,'y--');hold off 
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time1,handles.err_pos_v1,'b');%,handles.time,handles.t_position2,'r--');
    hold on
    plot(handles.time2,handles.err_pos_v2,'r');hold off 
    grid on;
    xlabel('Time (s)'); ylabel('Error pos Vehicle (m)');legend('Position'); set(legend,'Visible','off')

    text_my = strcat({'Positions blue '},handles.name1 ,{'& red '},handles.name2);
    set(handles.text1,'String',text_my)
    set(handles.radiobutton1,'Value',1)
    set(handles.radiobutton3,'Value',0)
    handles.v_a = 1;
    handles.current_joint =1;
end 
    
guidata(hObject, handles);

% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, ~, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.radiobutton3,'Value',0)
    handles.v_a=1;
    guidata(hObject, handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, ~, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.radiobutton1,'Value',0)
    handles.v_a=0;
    guidata(hObject, handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton3


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(~, ~, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.v_a == 1 
    
    i=1;
    tit = strcat('Vehicle','Estimate Parameters ');
    n = handles.n_v;
    
    if  handles.compare == 0 && handles.compare_now ==0
    
        gamma = handles.parameters(:,1:n);
        time = handles.time;
    
    elseif handles.compare == 1 || handles.compare_now ==1 %&& handles.w ==1
        
        gamma = handles.parameters1(:,1:n);
        time = handles.time1;

        gamma2 = handles.parameters2(:,1:n);
        time2 = handles.time2;
        
    end

elseif handles.v_a == 0
    
    i =handles.current_joint;
    numero = int2str(i);
    tit = strcat('Joint n',numero,'Estimate Parameters ');
    n = handles.n_a;
    
    if  handles.compare == 0 && handles.compare_now ==0
    
        gamma = handles.parameters(:,handles.n_v+1+n*(i-1):handles.n_v+n*(i));
        time = handles.time;
    
    elseif handles.compare == 1 || handles.compare_now ==1 %&& handles.w ==1
        
        gamma = handles.parameters1(:,handles.n_v+1+n*(i-1):handles.n_v+n*(i));
        time = handles.time1;

        gamma2 = handles.parameters2(:,handles.n_v+1+n*(i-1):handles.n_v+n*(i));
        time2 = handles.time2;
        
    end
    
end
        
    h=figure(i);
    set(h,'name',tit) % 'position',[50 320 1200 600],
    
    if mod(n,2)  ==  0
        n_2 = double(int8(n/2));
    else
        n_2 = double(int8((n+1)/2));
    end    
   
    for j = 1:n
        
        subplot(n_2,2,j)
        plot(time,gamma(:,j)); grid on;
        
        if  handles.compare == 1 || handles.compare_now ==1
        
            hold on
            plot(time2,gamma2(:,j),'g'); grid on;
            hold off
            
        end
    
    end

function edit1_Callback(hObject, ~, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
str = get(hObject,'String');
set(handles.edit1,'String',str);
handles.name_file = str;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, ~, ~)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, ~, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.compare_now == 1
    handles.compare_now =0;
    evalin('base','clear');
    assignin('base','name','matlab.mat');
    evalin('base','load(name)');
    evalin('base','clear name');
    set(handles.pushbutton18,'Backgroundcolor',[0.8 0.8 0.8])
    
    handles.time        = evalin('base','time');
    handles.pos_v       = evalin('base','PosV');
    handles.ori_v       = evalin('base','OriV');
    handles.velLin_v    = evalin('base','VelLinV');
    handles.velAng_v    = evalin('base','VelAngV');
    handles.pos_j       = evalin('base','PosJ');
    handles.vel_j       = evalin('base','VelJ');
    handles.end_j       = evalin('base','PoseEnd');
    handles.torque_arm  = evalin('base','torqueM');
    handles.parameters  = evalin('base','Theta');

    handles.pos_v_des   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des   = evalin('base','Pos_jdes');
    handles.vel_j_des   = evalin('base','Vel_jdes');

    handles.thrust      = evalin('base','ForceV(:,1:3)');
    handles.torque      = evalin('base','ForceV(:,4:6)');    
 
    
    for i = 1: length(handles.time)
        handles.err_pos_v(i,:) = norm(handles.pos_v_des(i,:)-handles.pos_v(i,:) );
    end
    
    axes(handles.axes1);
    plot(handles.time,handles.pos_v,handles.time,handles.pos_v_des,'r--');
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time,handles.err_pos_v,'r');%,handles.time,handles.t_position2,'r--');
    grid on;
    xlabel('Time (s)'); ylabel('Error pos Vehicle (m)');legend('Position'); set(legend,'Visible','off');

    set(handles.text1,'String','Positions')
    set(handles.radiobutton1,'Value',1)
    set(handles.radiobutton3,'Value',0)
    set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
    set(handles.pushbutton18,'Backgroundcolor',[0.8 0.8 0.8])
    handles.v_a = 1;
    handles.current_joint =1;
    handles.compare = 0;
    handles.compare_now = 0;
    
elseif handles.compare_now == 0

    handles.name1        = 'Now';
    handles.time1        = evalin('base','time');
    handles.pos_v1       = evalin('base','PosV');
    handles.ori_v1       = evalin('base','OriV');
    handles.velLin_v1    = evalin('base','VelLinV');
    handles.velAng_v1    = evalin('base','VelAngV');
    handles.pos_j1       = evalin('base','PosJ');
    handles.vel_j1       = evalin('base','VelJ');
    handles.end_j1       = evalin('base','PoseEnd');
    handles.torque_arm1  = evalin('base','torqueM');
    handles.parameters1  = evalin('base','Theta');

    handles.pos_v_des1   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des1   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des1   = evalin('base','Pos_jdes');
    handles.vel_j_des1   = evalin('base','Vel_jdes');    
    handles.thrust1      = evalin('base','ForceV(:,1:3)');
    handles.torque1      = evalin('base','ForceV(:,4:6)');    
 
    evalin('base','save();');
    
    for i = 1: length(handles.time1)
        handles.err_pos_v1(i,:) = norm(handles.pos_v_des1(i,:)-handles.pos_v1(i,:) );
    end
  
    evalin('base','clear')
    evalin('base','[FileName,PathName,FilterIndex] = uigetfile();');
    evalin('base','load(fullfile(PathName,FileName));');
    handles.name2        = evalin('base','FileName');
    handles.time2        = evalin('base','time');
    handles.pos_v2       = evalin('base','PosV');
    handles.ori_v2       = evalin('base','OriV');
    handles.velLin_v2    = evalin('base','VelLinV');
    handles.velAng_v2    = evalin('base','VelAngV');
    handles.pos_j2       = evalin('base','PosJ');
    handles.vel_j2       = evalin('base','VelJ');
    handles.end_j2       = evalin('base','PoseEnd');
    handles.torque_arm2  = evalin('base','torqueM');
    handles.parameters2  = evalin('base','Theta');

    handles.pos_v_des2   = evalin('base','Eta_d(:,1:3)');
    handles.vel_v_des2   = evalin('base','Eta_d(:,4:6)');
    handles.pos_j_des2   = evalin('base','Pos_jdes');
    handles.vel_j_des2   = evalin('base','Vel_jdes');

    handles.thrust2      = evalin('base','ForceV(:,1:3)');
    handles.torque2      = evalin('base','ForceV(:,4:6)');    
 
    for i = 1: length(handles.time2)
        handles.err_pos_v2(i,:) = norm(handles.pos_v_des2(i,:)-handles.pos_v2(i,:) );
    end
    
    set(handles.pushbutton18,'Backgroundcolor',[1 0 0])
    handles.compare_now = 1;
    handles.compare = 0;
    set(handles.pushbutton15,'Backgroundcolor',[0.8 0.8 0.8])
    
    axes(handles.axes1);
    plot(handles.time1,handles.pos_v1,'b',handles.time1,handles.pos_v_des1,'g--');
    hold on
    plot(handles.time2,handles.pos_v2,'r',handles.time2,handles.pos_v_des2,'y--');hold off 
    grid on;
    ylabel('Position Vehicle (m)');legend('Position','Trajectory'); set(legend,'Visible','off')
    axes(handles.axes2);
    plot(handles.time1,handles.err_pos_v1,'b');%,handles.time,handles.t_position2,'r--');
    hold on
    plot(handles.time2,handles.err_pos_v2,'r');hold off 
    grid on;
    xlabel('Time (s)'); ylabel('Error pos Vehicle (m)');legend('Position'); set(legend,'Visible','off')

    text_my = strcat({'Positions blue '},handles.name1 ,{'& red '},handles.name2);
    set(handles.text1,'String',text_my)
    set(handles.radiobutton1,'Value',1)
    set(handles.radiobutton3,'Value',0)
    handles.v_a = 1;
    handles.current_joint =1;
end 
    
guidata(hObject, handles);

