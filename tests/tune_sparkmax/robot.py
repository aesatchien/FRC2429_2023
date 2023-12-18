<xml version="1.0">
<dashboard>
	<widget field="steering_analog" type="Number" class="edu.wpi.first.smartdashboard.gui.elements.TextBox">
		<location x="21" y="23"/>
	</widget>
	<widget field="pedal_analog" type="Number" class="edu.wpi.first.smartdashboard.gui.elements.TextBox">
		<location x="15" y="11"/>
	</widget>
	<widget field="back" type="Boolean" class="edu.wpi.first.smartdashboard.gui.elements.TextBox">
		<location x="24" y="9"/>
	</widget>
	<widget field="forward" type="Boolean" class="edu.wpi.first.smartdashboard.gui.elements.TextBox">
		<location x="6" y="4"/>
	</widget>
</dashboard>
<live-window>
	<widget field="Ungrouped" type="LW Subsystem" class="edu.wpi.first.smartdashboard.livewindow.elements.LWSubsystem">
		<widget field="DigitalInput[2]" type="Digital Input" class="edu.wpi.first.smartdashboard.livewindow.elements.DigitalInputDisplay">
			<location x="6" y="16"/>
			<height>20</height>
			<width>89</width>
		</widget>
		<widget field="AnalogInput[1]" type="Analog Input" class="edu.wpi.first.smartdashboard.livewindow.elements.SingleNumberDisplay">
			<location x="6" y="36"/>
			<height>20</height>
			<width>89</width>
		</widget>
		<widget field="AnalogInput[0]" type="Analog Input" class="edu.wpi.first.smartdashboard.livewindow.elements.SingleNumberDisplay">
			<location x="6" y="56"/>
			<height>20</height>
			<width>89</width>
		</widget>
		<widget field="DigitalInput[3]" type="Digital Input" class="edu.wpi.first.smartdashboard.livewindow.elements.DigitalInputDisplay">
			<location x="6" y="76"/>
			<height>20</height>
			<width>89</width>
		</widget>
		<location x="21" y="7"/>
		<width>101</width>
		<height>102</height>
	</widget>
</live-window>
</xml>