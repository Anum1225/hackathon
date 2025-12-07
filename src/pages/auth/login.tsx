import React from 'react';
import Layout from '@theme/Layout';

export default function Login(): React.JSX.Element {
    return (
        <Layout title="Login">
            <div
                style={{
                    display: 'flex',
                    justifyContent: 'center',
                    alignItems: 'center',
                    height: '50vh',
                    fontSize: '20px',
                }}>
                <p>Redirecting to login...</p>
            </div>
        </Layout>
    );
}
