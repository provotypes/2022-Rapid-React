package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.*;
import java.io.File;
import java.io.*;
import com.google.gson.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

class Playlist {
    private List<String> music = new ArrayList<>();
    private List<String> current_playlist = new ArrayList<>();
    private Random r = new Random();
    private boolean loop = false;
    private int playlistPosition = 0;
    private boolean paused = true;
    private ArrayList<TalonFX> talonFXs;
    private Orchestra musicOrchestra;
    private String m_autoSelected;
    private String previous_selection;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final ShuffleboardTab musicTab = Shuffleboard.getTab("Music");


    public Playlist(WPI_TalonFX [] talonFXs) {
        this.talonFXs = new ArrayList<TalonFX>();
        for (int i = 0; i < talonFXs.length; i++) {
            this.talonFXs.add(talonFXs[i]);
        }
        musicOrchestra = new Orchestra(this.talonFXs);
        load();
    }
    private void createOption(String fileName, String displayName) {
        m_chooser.addOption(displayName, fileName);
    }

    
    private void load() {
        try {
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

    private List<String> getShuffled() {
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

    public boolean isPaused() {
        return paused;
    }

    public void shuffle() {
        current_playlist = getShuffled();
        playlistPosition = 0;
        play(current_playlist.get(playlistPosition));
    }

    public void update() {

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
    
    public void pause() {
        paused = true;
        musicOrchestra.pause();
    }

    public void resume() {
        paused = false;
        musicOrchestra.play();
    }

    public void play(String fileName) {
        System.out.println("Now playing: " + fileName);
        musicOrchestra.loadMusic(fileName);
        paused = false;
        musicOrchestra.play();
    }
    
    public void play() { resume(); }
    public boolean getLooping() { return loop; }
    public void setLooping(boolean b) { loop = b; }

}
