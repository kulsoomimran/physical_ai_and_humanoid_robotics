import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Simple SVG Icons
const BookIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M4 19.5V4.5C4 3.4 4.9 2.5 6 2.5H18C19.1 2.5 20 3.4 20 4.5V19.5C20 20.6 19.1 21.5 18 21.5H6C4.9 21.5 4 20.6 4 19.5Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M8 7.5H16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M8 11.5H16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M8 15.5H12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const RobotIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="3" y="7" width="18" height="12" rx="2" stroke="currentColor" strokeWidth="2"/>
    <path d="M8 7V5C8 3.89543 8.89543 3 10 3H14C15.1046 3 16 3.89543 16 5V7" stroke="currentColor" strokeWidth="2"/>
    <path d="M8 19V21" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
    <path d="M16 19V21" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
    <circle cx="9" cy="12" r="1" fill="currentColor"/>
    <circle cx="15" cy="12" r="1" fill="currentColor"/>
  </svg>
);

const AIIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2L15.09 8.26L22 9.27L17 14.14L18.18 21.02L12 17.77L5.82 21.02L7 14.14L2 9.27L8.91 8.26L12 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const CodeIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M7 8L3 12L7 16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M17 8L21 12L17 16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M14 4L10 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
  </svg>
);

const BrainIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 15C12 15 15 12.5 15 10C15 6.7 13.5 5 12 5C10.5 5 9 6.7 9 10C9 12.5 12 15 12 15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M12 15C12 15 9 17.5 9 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M12 15C12 15 15 17.5 15 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M8 12C8 12 7 10.5 7 9" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M16 12C16 12 17 10.5 17 9" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const ChipIcon = () => (
  <svg className="icon-svg" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="4" y="4" width="16" height="16" rx="2" stroke="currentColor" strokeWidth="2"/>
    <path d="M9 9H9.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M15 9H15.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M9 15H9.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M15 15H15.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M9 12H15" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
  </svg>
);

// Floating background icons
const FloatingIcon = ({ icon, style }: { icon: ReactNode; style?: React.CSSProperties }) => (
  <div className={styles.floatingIcon} style={style}>
    {icon}
  </div>
);

function HomepageBackground() {
  const icons = [<BookIcon />, <RobotIcon />, <AIIcon />];
  const floats = Array.from({ length: 10 }).map((_, i) => ({
    icon: icons[i % icons.length],
    style: {
      top: `${Math.random() * 80}%`,
      left: `${Math.random() * 90}%`,
      animationDuration: `${5 + Math.random() * 5}s`,
      animationDelay: `${Math.random() * 5}s`,
    },
  }));

  return <div className={styles.floatingBackground}>{floats.map((f, idx) => <FloatingIcon key={idx} icon={f.icon} style={f.style} />)}</div>;
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="text--center padding-horiz--md">
          <div className={clsx(styles.heroIcon, 'margin-bottom--lg')}>
            <RobotIcon />
          </div>
          <Heading as="h1" className="hero__title">{siteConfig.title}</Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link className="button button--secondary button--lg" to="/docs/intro">
               Start Reading - Introduction
            </Link>
            <Link className="button button--outline button--secondary button--lg margin-left--md" to="/docs/intro#book-structure">
              📚 Explore Modules
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function Section({ icon, title, children }: { icon: ReactNode; title: string; children: ReactNode }) {
  return (
    <section className={clsx('margin-vert--md', 'padding-vert--md')}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className="text--center padding-horiz--md">
              <div className={clsx(styles.sectionIcon, 'margin-bottom--sm')}>{icon}</div>
              <Heading as="h2" className="margin-bottom--md">{title}</Heading>
              <div className={styles.sectionContent}>{children}</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function WhoThisBookIsFor() {
  return (
    <Section icon={<RobotIcon />} title="Who This Book Is For">
      <div className="row">
        {[
          { icon: <AIIcon />, title: 'AI Researchers', desc: 'Understand AI systems embodied in physical systems.' },
          { icon: <CodeIcon />, title: 'Robotics Engineers', desc: 'Integrate AI techniques into robotic systems.' },
          { icon: <BrainIcon />, title: 'Graduate Students', desc: 'Comprehensive resource for robotics & AI studies.' },
        ].map((item, idx) => (
          <div key={idx} className="col col--4 margin-vert--md">
            <div className={clsx('card', styles.card)}>
              <div className={clsx(styles.cardIcon, 'margin-bottom--sm')}>{item.icon}</div>
              <h3>{item.title}</h3>
              <p>{item.desc}</p>
            </div>
          </div>
        ))}
      </div>
    </Section>
  );
}

function BookFeatures() {
  const features = [
    { icon: <BookIcon />, title: "Comprehensive Coverage", description: "From ROS 2 fundamentals to Vision-Language-Action systems" },
    { icon: <RobotIcon />, title: "Practical Implementation", description: "Hands-on exercises and code examples" },
    { icon: <AIIcon />, title: "Cutting-Edge AI", description: "Latest techniques in embodied AI & robotics" },
    { icon: <ChipIcon />, title: "Modern Frameworks", description: "NVIDIA Isaac, Gazebo, Unity & more" },
    { icon: <CodeIcon />, title: "Code Examples", description: "Practical implementations for each concept" },
    { icon: <BrainIcon />, title: "Embodied Intelligence", description: "How AI interacts with the physical world" },
  ];

  return (
    <Section icon={<ChipIcon />} title="What You'll Learn">
      <div className="row">
        {features.map((feature, idx) => (
          <div key={idx} className="col col--4 margin-vert--md">
            <div className={clsx('card', styles.featureCard)}>
              <div className={clsx(styles.featureIcon, 'margin-bottom--sm')}>{feature.icon}</div>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          </div>
        ))}
      </div>
    </Section>
  );
}

function FinalCTA() {
  return (
    <Section icon={<AIIcon />} title="Ready to Dive In?">
      <p>Begin your journey into robotics & embodied AI. This book provides knowledge and tools to build the future.</p>
      <div className={styles.buttons}>
        <Link className="button button--primary button--lg" to="/docs/intro">Start Reading Now</Link>
      </div>
    </Section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout title="Physical AI & Humanoid Robotics" description="Comprehensive Guide to Physical AI and Humanoid Robotics">
      <HomepageBackground />
      <HomepageHeader />
      <main>
        <Section icon={<BookIcon />} title="Bridging the Gap Between AI and Physical Systems">
          <p>This comprehensive guide explores the intersection of AI and robotics, focusing on how embodied systems learn, adapt, and interact with the physical world.</p>
        </Section>
        <WhoThisBookIsFor />
        <BookFeatures />
        <FinalCTA />
      </main>
    </Layout>
  );
}
