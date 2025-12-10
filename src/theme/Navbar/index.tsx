import React from 'react';
import Navbar from '@theme-original/Navbar';
import SearchBar from '@site/src/components/SearchBar';

export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      <div style={{ position: 'fixed', top: '12px', left: '450px', zIndex: 1000 }}>
        <SearchBar />
      </div>
    </>
  );
}
