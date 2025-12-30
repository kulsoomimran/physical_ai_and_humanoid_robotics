import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Translate, { translate } from '@docusaurus/Translate';

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
          <Heading as="h1" className="hero__title">
            <Translate id="pages.index.title" description="The main title on the home page">
              {siteConfig.title}
            </Translate>
          </Heading>
          <p className="hero__subtitle">
            <Translate id="pages.index.tagline" description="The tagline on the home page">
              {siteConfig.tagline}
            </Translate>
          </p>
          <div className={styles.buttons}>
            <Link className="button button--secondary button--lg" to="/docs/intro">
              <Translate id="pages.index.button.startReading" description="The start reading button text on the home page">
                Start Reading - Introduction
              </Translate>
            </Link>
            <Link className="button button--outline button--secondary button--lg margin-left--md" to="/docs/intro#book-structure">
              <Translate id="pages.index.button.exploreModules" description="The explore modules button text on the home page">
                📚 Explore Modules
              </Translate>
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
    <Section
      icon={<RobotIcon />}
      title={
        <Translate id="pages.index.section.title.whoFor" description="The title for the who this book is for section on the home page">
          Who This Book Is For
        </Translate>
      }
    >
      <div className="row">
        {[
          {
            icon: <AIIcon />,
            title: (
              <Translate id="pages.index.card.title.aiResearchers" description="The title for AI Researchers card">
                AI Researchers
              </Translate>
            ),
            desc: (
              <Translate id="pages.index.card.content.aiResearchers" description="The content for AI Researchers card">
                Understand AI systems embodied in physical systems.
              </Translate>
            )
          },
          {
            icon: <CodeIcon />,
            title: (
              <Translate id="pages.index.card.title.roboticsEngineers" description="The title for Robotics Engineers card">
                Robotics Engineers
              </Translate>
            ),
            desc: (
              <Translate id="pages.index.card.content.roboticsEngineers" description="The content for Robotics Engineers card">
                Integrate AI techniques into robotic systems.
              </Translate>
            )
          },
          {
            icon: <BrainIcon />,
            title: (
              <Translate id="pages.index.card.title.graduateStudents" description="The title for Graduate Students card">
                Graduate Students
              </Translate>
            ),
            desc: (
              <Translate id="pages.index.card.content.graduateStudents" description="The content for Graduate Students card">
                Comprehensive resource for robotics & AI studies.
              </Translate>
            )
          },
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
    {
      icon: <BookIcon />,
      title: (
        <Translate id="pages.index.feature.title.comprehensiveCoverage" description="The title for Comprehensive Coverage feature">
          Comprehensive Coverage
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.comprehensiveCoverage" description="The content for Comprehensive Coverage feature">
          From ROS 2 fundamentals to Vision-Language-Action systems
        </Translate>
      )
    },
    {
      icon: <RobotIcon />,
      title: (
        <Translate id="pages.index.feature.title.practicalImplementation" description="The title for Practical Implementation feature">
          Practical Implementation
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.practicalImplementation" description="The content for Practical Implementation feature">
          Hands-on exercises and code examples
        </Translate>
      )
    },
    {
      icon: <AIIcon />,
      title: (
        <Translate id="pages.index.feature.title.cuttiungEdgeAI" description="The title for Cutting Edge AI feature">
          Cutting-Edge AI
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.cuttiungEdgeAI" description="The content for Cutting Edge AI feature">
          Latest techniques in embodied AI & robotics
        </Translate>
      )
    },
    {
      icon: <ChipIcon />,
      title: (
        <Translate id="pages.index.feature.title.modernFrameworks" description="The title for Modern Frameworks feature">
          Modern Frameworks
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.modernFrameworks" description="The content for Modern Frameworks feature">
          NVIDIA Isaac, Gazebo, Unity & more
        </Translate>
      )
    },
    {
      icon: <CodeIcon />,
      title: (
        <Translate id="pages.index.feature.title.codeExamples" description="The title for Code Examples feature">
          Code Examples
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.codeExamples" description="The content for Code Examples feature">
          Practical implementations for each concept
        </Translate>
      )
    },
    {
      icon: <BrainIcon />,
      title: (
        <Translate id="pages.index.feature.title.embodiedIntelligence" description="The title for Embodied Intelligence feature">
          Embodied Intelligence
        </Translate>
      ),
      description: (
        <Translate id="pages.index.feature.content.embodiedIntelligence" description="The content for Embodied Intelligence feature">
          How AI interacts with the physical world
        </Translate>
      )
    },
  ];

  return (
    <Section
      icon={<ChipIcon />}
      title={
        <Translate id="pages.index.section.title.whatYouLearn" description="The title for the what you'll learn section on the home page">
          What You'll Learn
        </Translate>
      }
    >
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
    <Section
      icon={<AIIcon />}
      title={
        <Translate id="pages.index.section.title.readyToDive" description="The title for the ready to dive in section on the home page">
          Ready to Dive In?
        </Translate>
      }
    >
      <p>
        <Translate id="pages.index.section.content.readyToDive" description="The content for the ready to dive in section on the home page">
          Begin your journey into robotics & embodied AI. This book provides knowledge and tools to build the future.
        </Translate>
      </p>
      <div className={styles.buttons}>
        <Link className="button button--primary button--lg" to="/docs/intro">
          <Translate id="pages.index.button.startReading" description="The start reading button text on the home page">
            Start Reading Now
          </Translate>
        </Link>
      </div>
    </Section>
  );
}

export default function Home(): ReactNode {
  const title = translate({
    id: 'pages.index.title',
    message: 'Physical AI & Humanoid Robotics',
    description: 'The main title on the home page'
  });

  const description = translate({
    id: 'pages.index.tagline',
    message: 'Comprehensive Guide to Physical AI and Humanoid Robotics',
    description: 'The tagline on the home page'
  });

  return (
    <Layout title={title} description={description}>
      <HomepageBackground />
      <HomepageHeader />
      <main>
        <Section
          icon={<BookIcon />}
          title={
            <Translate id="pages.index.section.title.bridgingGap" description="The title for the bridging gap section on the home page">
              Bridging the Gap Between AI and Physical Systems
            </Translate>
          }
        >
          <p>
            <Translate id="pages.index.section.content.bridgingGap" description="The content for the bridging gap section on the home page">
              This comprehensive guide explores the intersection of AI and robotics, focusing on how embodied systems learn, adapt, and interact with the physical world.
            </Translate>
          </p>
        </Section>
        <WhoThisBookIsFor />
        <BookFeatures />
        <FinalCTA />
      </main>
    </Layout>
  );
}
