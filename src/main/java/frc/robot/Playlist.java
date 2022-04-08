package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.*;
import java.io.File;
import java.io.*;
import com.google.gson.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

class Playlist {
    private static List<String> music = new ArrayList<>();
    private static List<String> current_playlist = new ArrayList<>();
    private static Random r = new Random();
    private static boolean loop = false;
    private static int playlistPosition = 0;
    private static boolean paused = true;
    private static WPI_TalonFX motor1; // = new WPI_TalonFX(9);
    private static WPI_TalonFX motor2; // = new WPI_TalonFX(10);
    private static WPI_TalonFX motor4; // = new WPI_TalonFX(11);
    private static WPI_TalonFX motor3; // = new WPI_TalonFX(12);
    private static Orchestra musicOrchestra = new Orchestra();
    private static String m_autoSelected;
    private static String previous_selection;
    private static final SendableChooser<String> m_chooser = new SendableChooser<>();
    private static final ShuffleboardTab musicTab = Shuffleboard.getTab("Music");


    public static void setMotors(List<WPI_TalonFX> motors) {
        motor1 = motors.get(0);
        motor2 = motors.get(1);
        motor3 = motors.get(2);
        motor4 = motors.get(3);
    }
    private static void createOption(String fileName, String displayName) {
        m_chooser.addOption(displayName, fileName);
    }

    
    public static void load() {
        try {
            musicOrchestra.addInstrument(motor1);
            musicOrchestra.addInstrument(motor2);
            musicOrchestra.addInstrument(motor3);
            musicOrchestra.addInstrument(motor4);
            File file = new File(Filesystem.getDeployDirectory().getPath().concat("/Playlist.json"));
            JsonElement conf = JsonParser.parseReader(new FileReader(file));
            Map<String, String> _music = new HashMap<>();
            JsonObject Music = conf.getAsJsonObject().get("music").getAsJsonObject();
            //JsonObject Ignore = conf.getAsJsonObject().get("ignore").getAsJsonObject();
            //Set<Map.Entry<String, JsonElement>> Ignoring = Ignore.entrySet();
            //List<String> IgnoreTitles = new ArrayList<>();
            
            //for (Map.Entry<String, JsonElement> i : Ignoring) {
            //    IgnoreTitles.add(i.getKey());
            //}
            m_chooser.setDefaultOption("Pause", "<pause>");
            m_chooser.addOption("Shuffle", "<shuffle>");
            for (Map.Entry<String, JsonElement> configObj : Music.entrySet()) {
                //if (IgnoreTitles.contains(configObj.getKey())) { continue; }
                String fileName = configObj.getKey() + ".chrp";
                String displayName = configObj.getValue().toString().replaceAll("\"", "");
                //  ?  toString method might need to be different
                music.add(fileName);
                _music.put(fileName, displayName);
                createOption(fileName, displayName);
            }
            // SmartDashboard.putData("Song Choices", m_chooser);
            musicTab.add(m_chooser).withSize(3, 1);

            current_playlist = List.copyOf(music);

            musicOrchestra.loadMusic("Pirates.chrp");

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static List<String> getShuffled() {
        List<String> shuffled = new ArrayList<>();
        List<String> musicCopy = List.copyOf(music);

        for (String fileName : musicCopy) {
            int i = 0;
            if (shuffled.size() > 0) {
                i = r.nextInt(shuffled.size());
            }
            shuffled.add(i, fileName);
        }

        return shuffled;
    }

    public static boolean isPaused() {
        return paused;
    }

    public static void shuffle() {
        current_playlist = getShuffled();
        playlistPosition = 0;
        play(current_playlist.get(playlistPosition));
    }

    public static void update() {

        previous_selection = m_autoSelected;
        m_autoSelected = m_chooser.getSelected();

        if (m_autoSelected != previous_selection) {
            if (m_autoSelected == "<shuffle>") {
                shuffle();
            } else if (m_autoSelected == "<pause>") {
                pause();
            } else {
                if (previous_selection == "<pause>") {
                    play();
                } else {
                    play(m_autoSelected);
                }
            }
        }

        boolean playing = musicOrchestra.isPlaying();
        if ((!playing) && (!paused)) {
            System.out.println("Playing next...");
            //playlistPosition++;
            if (playlistPosition >= current_playlist.size()) {
                playlistPosition = 0;
                if (loop) { paused = true; }
            }

            play(current_playlist.get(playlistPosition));

        }
    }
    
    public static void pause() {
        paused = true;
        musicOrchestra.pause();
    }

    public static void resume() {
        paused = false;
        musicOrchestra.play();
    }

    public static void play(String fileName) {
        System.out.println("Now playing: " + fileName);
        musicOrchestra.loadMusic(fileName);
        paused = false;
        musicOrchestra.play();
    }
    
    public static void play() { resume(); }
    public static boolean getLooping() { return loop; }
    public static void setLooping(boolean b) { loop = b; }

}
