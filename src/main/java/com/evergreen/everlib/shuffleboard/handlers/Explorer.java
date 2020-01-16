package com.evergreen.everlib.shuffleboard.handlers;

import java.util.Stack;

/**
 * Explorer
 */
public class Explorer {

    private String m_workingDirectory;
    private Stack<String> m_directoryStack;

    public Explorer() {
        m_workingDirectory = "";
    }

    public Explorer(String path) {
        this();
        cd(path);
    }

    public Explorer cd(String path) {
        if (path.startsWith("/")) {
            m_workingDirectory = "";
            path = path.substring(0, path.length());
        }

        for (String folder : path.split("/")) {
            if (folder == "..") {
                m_workingDirectory = m_workingDirectory.substring(
                    0, m_workingDirectory.lastIndexOf('/'));
            }

            else if (folder != ".") {
                m_workingDirectory += "/";
                m_workingDirectory += folder;
            }
        }

        return new Explorer(m_workingDirectory);
    }

    public String pwd() {
        return "/" + m_workingDirectory;
    }

    public void pushd(String path) {
        m_directoryStack.push(pwd());
        cd(path); 
    }

    public void popd() {
        cd(m_directoryStack.pop());
    }
}