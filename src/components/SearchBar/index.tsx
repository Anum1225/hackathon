import React, { useState, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

const pages = [
  { title: 'Introduction & Course Overview', path: '/docs/intro', keywords: 'physical ai humanoid robotics course overview introduction' },
  { title: 'Hardware Lab Setup', path: '/docs/hardware-lab', keywords: 'hardware gpu rtx jetson workstation requirements' },
  { title: 'Week 1-2: Introduction to Physical AI', path: '/docs/week-01-02-intro', keywords: 'embodied intelligence sensors humanoid landscape' },
  { title: 'Week 3-5: ROS 2 Fundamentals', path: '/docs/week-03-05-ros2', keywords: 'ros2 nodes topics services actions packages' },
  { title: 'Week 6-7: Robot Simulation', path: '/docs/week-06-07-gazebo', keywords: 'gazebo urdf sdf physics simulation unity' },
  { title: 'Week 8-10: NVIDIA Isaac Platform', path: '/docs/week-08-10-isaac', keywords: 'isaac sim vslam nvblox nav2 nvidia' },
  { title: 'Week 11-12: Humanoid Development', path: '/docs/week-11-12-humanoid', keywords: 'kinematics dynamics bipedal locomotion manipulation' },
  { title: 'Week 13: Conversational Robotics', path: '/docs/week-13-conversational', keywords: 'gpt whisper voice multi-modal interaction' },
];

export default function SearchBar() {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const inputRef = useRef<HTMLInputElement>(null);
  const history = useHistory();

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setIsOpen(true);
      }
      if (e.key === 'Escape') {
        setIsOpen(false);
        setQuery('');
        setResults([]);
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  useEffect(() => {
    if (query.trim()) {
      const filtered = pages.filter(page => 
        page.title.toLowerCase().includes(query.toLowerCase()) ||
        page.keywords.toLowerCase().includes(query.toLowerCase())
      );
      setResults(filtered);
    } else {
      setResults([]);
    }
  }, [query]);

  const handleSearch = (e: React.FormEvent) => {
    e.preventDefault();
    if (results.length > 0) {
      history.push(results[0].path);
      setIsOpen(false);
      setQuery('');
      setResults([]);
    }
  };

  const handleResultClick = (path: string) => {
    history.push(path);
    setIsOpen(false);
    setQuery('');
    setResults([]);
  };

  return (
    <>
      <button className={styles.searchTrigger} onClick={() => setIsOpen(true)}>
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="11" cy="11" r="8" />
          <path d="m21 21-4.35-4.35" />
        </svg>
        <span>Search...</span>
        <kbd className={styles.kbd}>Ctrl K</kbd>
      </button>

      {isOpen && (
        <div className={styles.overlay} onClick={() => { setIsOpen(false); setQuery(''); setResults([]); }}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <form onSubmit={handleSearch}>
              <div className={styles.searchBox}>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="11" cy="11" r="8" />
                  <path d="m21 21-4.35-4.35" />
                </svg>
                <input
                  ref={inputRef}
                  type="text"
                  placeholder="Search documentation..."
                  value={query}
                  onChange={(e) => setQuery(e.target.value)}
                  className={styles.input}
                />
                <button type="button" onClick={() => { setIsOpen(false); setQuery(''); setResults([]); }} className={styles.closeBtn}>
                  ESC
                </button>
              </div>
            </form>
            {results.length > 0 && (
              <div className={styles.results}>
                {results.map((result, idx) => (
                  <div key={idx} className={styles.resultItem} onClick={() => handleResultClick(result.path)}>
                    <div className={styles.resultTitle}>{result.title}</div>
                    <div className={styles.resultPath}>{result.path}</div>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
}
